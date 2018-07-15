#include <avr/sleep.h>
#include <avr/interrupt.h>

const int wakePin                     = 3;
int awakeLedPin = 1;            // LED connected to digital pin 0. Shows when the circuit is awake
int interruptPin = 2;      // LED to show the action of a interrupt


void setup() {
  pinMode(awakeLedPin, OUTPUT);         // sets the digital pin as output
  pinMode(interruptPin, OUTPUT);   // sets the digital pin as output
  pinMode(wakePin, INPUT_PULLUP);        // sets the digital pin as input
    } // setup

void sleepNow() {

    GIMSK |= _BV(PCIE);                     // Enable Pin Change Interrupts
    PCMSK |= _BV(PCINT3);                   // Use PB3 as interrupt pin
    ADCSRA &= ~_BV(ADEN);                   // ADC off
    
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);    // replaces above statement
    sleep_enable();                         // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
    sei();                                  // Enable interrupts
    sleep_mode();                          // here the device is actually put to sleep!!


    sleep_disable();                        // Clear SE bit
    cli();                                  // Disable interrupts
    PCMSK &= ~_BV(PCINT3);                  // Turn off PB3 as interrupt pin
    ADCSRA |= _BV(ADEN);                    // ADC on

    //sei();                                  // Enable interrupts
    delay(250);                           // wait 2 sec. so humans can notice the interrupt LED to show the interrupt is handled
    digitalWrite (interruptPin, LOW);       // turn off the interrupt LED
    } // sleep

ISR(PCINT0_vect) {
    // This is called when the interrupt occurs, but I don't need to do anything in it
    }

void wakeUpNow(){        // here the interrupt is handled after wakeup

  //execute code here after wake-up before returning to the loop() function
  // timers and code using timers (serial.print and more...) will not work here.
 digitalWrite(interruptPin, HIGH);
}


void loop(){
  digitalWrite(awakeLedPin, HIGH);            // sets the LED on
  delay(3000);                           // waits for a second
  sleepNow();                      // sleep function called here
}
