/*
   Compiled with MicroCore: 980 bytes. (Not working..)
   Compiled for Attiny85: 956 bytes.
                          5244 bytes with debug.
How to read this code:
1. Don't. It's ugly.
2. When "pin" is referred in code (or text) it's the Atari side of things. The ATtiny's pins are always referred to as PBX. 


   Change Log:
   01/05/2021 - JoeFloyd-on-Github
   1) Enabled serial output on LEGACY board 
      * Used to debug, but leaving enabled on boards being provided to end users just in case they want to see output from the board
   2) Changed SELECT_PORT to PORTB rather than DDRB
   3) Removed command mode which could be entered at board startup
   4) Changed location of state variable in flash.
   5) Changed logic of state change and flush to flash  
   6) Disabled pulse detection and state update due to inablilty to suppress spurious pulse events
     * I tried to prevent spurious pulses by using a pull-down resistor of 10k on the pulse pin
     * This did reduce the number of pulse events seen, but also broke detection of the mode switch physical button
     * More investigation of the pulse detection logic is needed in order to re-enable software control of mode switching
     * Also, on Atari STE running MTOS and loading Lemmings the video output is corrupted due to incompatibility between MTOS and Lemmings
     * The corrupted video output resulted in many many spurious pulse events and eventually resulted in un-expected mode switch
     * This shows that the pulse length is not sufficient to determine software state change.  It would be better to use multiple pulses
     * with specific lenthgs to encode information which could then be verified as a unique command
*/

#define LEGACY // Uncomment this if your device does not have a ISP header.
//#define ALT_DETECT

#define DEBUG

// Timings.
#define switchTime 250000
#define saveTime 2000000
#define bootWait 4000 // Wait for computer to stabilize, 3-4s seems ok.
#define pulseThreshold 50000
#define SELECT_PORT PORTB
//#define SELECT_PORT DDRB

#define SAVE_LOCATION 0x1830  //6192

#include <util/delay.h>
#include <EEPROM.h>

#ifdef LEGACY
#define buttonPin PB0       // Connected to switch
#define selectPin PB1     // TS5V330 pin 1
#define pulsePin PB2       // VSync
#define serialRX PB3
#define serialTX PB4
#else
#defien serialRX PB0
#define serialTX PB1
#define pulsePin PB2       // VSync
#define selectPin PB3     // TS5V330 pin 1
#define buttonPin PB4       // Connected to switch
#endif

// Now that the pins are set, undefine LEGACY so that the ifdef's around the DEBUG statements are happy
#undef LEGACY

#if defined(DEBUG) 
#include <SoftwareSerial.h>
SoftwareSerial mySerial(serialRX, serialTX); // RX, TX
#endif

unsigned long buttonPressedTime;

byte state;
byte state_changed = 0;
byte tosState;

#ifdef DEBUG
bool done = false;
String buff;
char data;
int i = 0;
#endif

unsigned long pulse = 0;

void setup() {
// IDEA: check if a jumper is placed on the ISP header to enter some kind of other operations mode. 

  // Set state to an invalid value to make sure the read from flash works
  state = 3;

  // Do the hardware setup for the Atari before setting up the serial port.
  state = EEPROM.read(SAVE_LOCATION);
  PORTB |= bit(buttonPin); // Input pullup.
  // Instead of sending +5V to the Atari, let the pullup on pin4 handle the high part.
  bitWrite(SELECT_PORT, selectPin, state);  
  
#if defined (DEBUG) && !defined(LEGACY)
  mySerial.begin(9600);
  mySerial.println("Boot");
#endif

#if defined (DEBUG) && !defined(LEGACY)
  mySerial.print("Reading state from flash: Saved state ");
  mySerial.println(state);
#endif
  
// Instead of sending +5V to the Atari, let the pullup on pin4 handle the high part.
  bitWrite(DDRB, selectPin, state); 

#if defined (DEBUG) && !defined(LEGACY)
  mySerial.print("Waiting for ");
  mySerial.print(bootWait);
  mySerial.println(" milliseconds.");
#endif
  _delay_ms(bootWait);
  state_changed = 0;

#if defined (DEBUG) && !defined(LEGACY)
  mySerial.println("Loop.");
#endif
}




#ifdef DEBUG
void clearBuff() {
  buff = "";
  i = 0;
  mySerial.print("> ");
}



void printHelp() {
  mySerial.println("Help");
  mySerial.println("-----------");
  mySerial.println("b       - Simulate button press.");
  mySerial.println("p       - Print information.");
  mySerial.println("q       - Quit prompt.");
  mySerial.println("s <num> - Set pulse threashold.");

}
#endif

void toggle_state(void)
{
  if (state == 0)
  {
    state = 1;
  }
  else
  {
    state = 0;
  }
  state_changed = 1;
}

void loop() {

  byte save_now = 0;
  /*
     Detection..
  */
  pulse = 0;
#ifdef ALT_DETECT
  while ( pulse < 10000 ) // 10000 since sometimes you'd get 14 from getPulse();
    pulse = getPulse();
#else


  // Check for pulsePin going high.  This is to allow for Atari ST program control over the video mode
  while ( ! pulse )
    pulse = pulseIn(pulsePin, HIGH, 1000000L);


  if ( 0 != pulse )
  { 
    if (pulse < pulseThreshold)
    {
       //mySerial.println("Pulse detected, but below pulse threshold");
       //mySerial.print("Pulse: ");
       //mySerial.println(pulse);
    }
    else
    {
#if defined (DEBUG) && !defined(LEGACY)
      mySerial.println("Pulse detected, and above pulse threshold");
      mySerial.print("Pulse: ");
      mySerial.println(pulse);
#endif
      save_now = 1;
      toggle_state();
    }

  }

  #endif

  // Check for button press.  This is to allow for manual user control over the video mode
  // Short press (> 0.25s and < 2.0s) change mode
  // Long press (>= 2.0s) save mode

  if ( !bitRead(PINB, buttonPin) ) 
  {
    buttonPressedTime = micros();
    while ( ! bitRead(PINB, buttonPin)); // Ugly debounce
    if ( micros() > saveTime  + buttonPressedTime ) 
    {
#if defined (DEBUG) && !defined(LEGACY)
      mySerial.println("Long Button Press");
#endif
      save_now = 1;
    } 
    else if ( micros() > buttonPressedTime + switchTime ) 
    {
#if defined (DEBUG) && !defined(LEGACY)
      mySerial.println("Short Button Press");
#endif
    // Toggle the state
    save_now = 1;
    toggle_state();
    }
  }

  // Check to see if the state should be saved due to a long button press
  if ( 1 == save_now )
  {
 #if defined (DEBUG) && !defined(LEGACY)
      mySerial.println("Save");
      mySerial.print("Value: ");
      mySerial.println(state);
#endif
      save_now = 0;
      EEPROM.write(SAVE_LOCATION, state);
  }

  // If the state has changed, update the state of the selectPin
  if ( 1 == state_changed ) 
  {
#if defined (DEBUG) && !defined(LEGACY)
    //mySerial.println("State change");
    //mySerial.print("New state: ");
    //mySerial.println(state);
#endif
    bitWrite(SELECT_PORT, selectPin, state);
    // Reset the state_changed flag.
    state_changed = 0;
    _delay_ms(1000); // Wait for computer to change video modes
  }
   
}

unsigned long getPulse() {
  pulse = bitRead(PINB, pulsePin);
  while ( bitRead(PINB, pulsePin) == pulse);
  unsigned long pstart = micros();
  while ( bitRead(PINB, pulsePin) != pulse);
  return ( micros() - pstart);
}
