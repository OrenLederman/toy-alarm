#include "PinChangeInterrupt.h"

#include <avr/sleep.h>
#include <avr/interrupt.h>

const int keyPin = 0;         // Pin pulled up when key A is pressed on keyfab
const int keyPin2 = 2;        // Pin pulled up when key B is pressed on keyfab

int mosfetPin = 3;            // Pin used for turning on mosfet
int soundTriggerPin = 1;      // Pin used for triggering MP3 player
int soundTriggerPin2 = 4;     // Pin used for triggering MP3 player

volatile int pinToPull = -1;


void setup() {

  pinMode(keyPin, INPUT);
  pinMode(keyPin2, INPUT);
  
  pinMode(mosfetPin, OUTPUT);         // sets the digital pin as output

  // Setting up the mosfet pin to HIGH (P-CHANNEL)
  digitalWrite (mosfetPin, HIGH);       

  // Setting sound triggers to INPUT so the mp3 player doesn't suck
  // power though these pins at startup
  pinMode(soundTriggerPin, INPUT);  
  pinMode(soundTriggerPin2, INPUT); 
  delay(500);

  // Set sound triggers to output, and set to high (need to pull down to trigger the player)
  pinMode(soundTriggerPin, OUTPUT);   // sets the digital pin as output
  pinMode(soundTriggerPin2, OUTPUT);   // sets the digital pin as output
  delay(100);
  digitalWrite (soundTriggerPin, LOW);  // pins set to low so they dont feed the mp3 player when it's uspposed to be off
  digitalWrite (soundTriggerPin2, LOW); // pins set to low so they dont feed the mp3 player when it's uspposed to be off 

  
} // setup

void sleep() {

    //GIMSK |= _BV(PCIE);                     // Enable Pin Change Interrupts
    attachPCINT(digitalPinToPCINT(keyPin), markTriggered, CHANGE);
    attachPCINT(digitalPinToPCINT(keyPin2), markTriggered2, CHANGE);


    ADCSRA &= ~_BV(ADEN);                   // ADC off
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);    // replaces above statement
    
    sleep_enable();                         // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
    sei();                                  // Enable interrupts
    sleep_cpu();                            // sleep

    cli();                                  // Disable interrupts
    detachPCINT(digitalPinToPCINT(keyPin));
    detachPCINT(digitalPinToPCINT(keyPin2));
    sleep_disable();                        // Clear SE bit
    ADCSRA |= _BV(ADEN);                    // ADC on
    sei();                                  // Enable interrupts
    } // sleep

void markTriggered(void) {
  pinToPull = soundTriggerPin;
}

void markTriggered2(void) {
  pinToPull = soundTriggerPin2;
}


void loop() {
  sleep();
  if (pinToPull >= 0) {
    // Play sound
    digitalWrite(mosfetPin, LOW);      // Turn on power to mp3 player
    digitalWrite (soundTriggerPin, HIGH);  // pins set to high
    digitalWrite (soundTriggerPin2, HIGH); // pins set to high
    
    delay(300);    
    digitalWrite(pinToPull, LOW);  // Trigger sound
    delay(100);
    digitalWrite (soundTriggerPin, HIGH);  // stop trigger
    digitalWrite (soundTriggerPin2, HIGH); // stop trigger
    
    delay(1200);                          // let it play for 1 sec                     
    digitalWrite(mosfetPin, HIGH);       // Turn off power
    digitalWrite (soundTriggerPin, LOW);  // pins set to low so they dont feed the mp3 player when it's uspposed to be off
    digitalWrite (soundTriggerPin2, LOW); // pins set to low so they dont feed the mp3 player when it's uspposed to be off 
    

    // Reset stuff
    pinToPull = -1;
  }

}
