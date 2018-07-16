#include "PinChangeInterrupt.h"

#include <avr/sleep.h>
#include <avr/interrupt.h>

const int switchPin                     = 0;
const int switchPin2                     = 2;

int awakeLedPin = 1;            // LED connected to digital pin 0. Shows when the circuit is awake
int interruptLedPin = 3;      // LED to show the action of a interrupt
int interruptLedPin2 = 4;      // LED to show the action of a interrupt

volatile int pinToPull = -1;


void setup() {

  pinMode(switchPin, INPUT);
  pinMode(switchPin2, INPUT);
  
  pinMode(awakeLedPin, OUTPUT);         // sets the digital pin as output
  pinMode(interruptLedPin, OUTPUT);   // sets the digital pin as output
  pinMode(interruptLedPin2, OUTPUT);   // sets the digital pin as output

  digitalWrite (interruptLedPin, HIGH);       // turn off the interrupt LED
  digitalWrite (interruptLedPin2, HIGH);       // turn off the interrupt LED
  
  delay(500);
  
} // setup

void sleep() {

    //GIMSK |= _BV(PCIE);                     // Enable Pin Change Interrupts
    attachPCINT(digitalPinToPCINT(switchPin), markTriggered, CHANGE);
    attachPCINT(digitalPinToPCINT(switchPin2), markTriggered2, CHANGE);


    ADCSRA &= ~_BV(ADEN);                   // ADC off
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);    // replaces above statement
    digitalWrite(awakeLedPin, LOW); 
    
    sleep_enable();                         // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
    sei();                                  // Enable interrupts
    sleep_cpu();                            // sleep

    cli();                                  // Disable interrupts
    detachPCINT(digitalPinToPCINT(switchPin));
    detachPCINT(digitalPinToPCINT(switchPin2));
    sleep_disable();                        // Clear SE bit
    ADCSRA |= _BV(ADEN);                    // ADC on
    sei();                                  // Enable interrupts
    } // sleep

void markTriggered(void) {
  pinToPull = interruptLedPin;
}

void markTriggered2(void) {
  pinToPull = interruptLedPin2;
}


void loop() {
  sleep();
  if (pinToPull >= 0) {
    // Play sound
    digitalWrite(awakeLedPin, HIGH);      // Turn on power to mp3 player
    delay(300);    
    digitalWrite(pinToPull, LOW);  // Trigger sound
    delay(100);
    digitalWrite(pinToPull, HIGH); // Stop trigger
    delay(1000);                          // let it play for 1 sec                     
    digitalWrite(awakeLedPin, LOW);       // Turn off power

    // Reset stuff
    pinToPull = -1;
  }

}
