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

  digitalWrite (interruptLedPin, HIGH);       // turn on the interrupt indicator LED
  digitalWrite (interruptLedPin2, HIGH);       // turn on the interrupt indicator LED
  delay(500);
} // setup


void loop() {
    digitalWrite(awakeLedPin, HIGH);      // Turn on power to mp3 player
    delay(300);                         
    digitalWrite(interruptLedPin2, LOW);  // Trigger sound
    delay(100);                           
    digitalWrite(interruptLedPin2, HIGH); // Stop trigger
    delay(1000);                          // let it play for 1 sec                     
    digitalWrite(awakeLedPin, LOW);       // Turn off power
    delay(5000);                               
    
}
