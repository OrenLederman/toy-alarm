#include "PinChangeInterrupt.h"

#include <avr/sleep.h>
#include <avr/interrupt.h>

const int switchPin                     = 0;
const int switchPin2                     = 2;

int awakeLedPin = 1;            // LED connected to digital pin 0. Shows when the circuit is awake
int interruptLedPin = 3;      // LED to show the action of a interrupt
int interruptLedPin2 = 4;      // LED to show the action of a interrupt

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
    attachPCINT(digitalPinToPCINT(switchPin), blinkLed, CHANGE);
    attachPCINT(digitalPinToPCINT(switchPin2), blinkLed2, CHANGE);


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
    
    delay(500);                           // wait 2 sec. so humans can notice the interrupt LED to show the interrupt is handled
    digitalWrite (interruptLedPin, HIGH);       // turn off the interrupt LED
    digitalWrite (interruptLedPin2, HIGH);       // turn off the interrupt LED

    
    } // sleep

void blinkLed(void) {
  // Switch Led state
  digitalWrite(interruptLedPin, LOW);
}

void blinkLed2(void) {
  // Switch Led state
  digitalWrite(interruptLedPin2, LOW);
}


void loop() {
    digitalWrite(awakeLedPin, HIGH);            // sets the LED on
    delay(1000);                           // waits for a second
   
    sleep();
}
