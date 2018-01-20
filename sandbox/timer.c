/************************************************************************/
/* Simple timer-based ISR for a desired frequency
/* (here, target frequency is 2Hz)
/************************************************************************/

#define F_CPU 16000000L

#include <avr/io.h>
#include <avr/interrupt.h> // Needed for cli() and sei()
#include <util/delay.h>

volatile int ledState = 0;

void initISRTimer() {
  // Initialise the Timer 1 registers
  TCCR1A = 0;
  TCCR1B = 0;
  
  // We use TIMER1 in CTC mode
  TCCR1B |= (1 << WGM12);
  
  // Set prescaler value to 256
  TCCR1B |= (1 << CS12);
  
  // Enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  
  // Set compare match register to desired value for 2Hz frequency
  OCR1A = 31250;  
}

int main(void) {
  // Set the mode for pin 5 on PORTB to "output"
  DDRB |= 0B00100000;
  
  // Set up the timer for the interrupt
  // First, disable all interrupts
  cli();
  initISRTimer();
  // Enable all interrupts again
  sei();
  
  while (1) {
  }
}


// Invoked every 500ms, toggles the state of the LED
ISR(TIMER1_COMPA_vect) {
  if (ledState == 1) {
    PORTB |= 0B00100000;
  } else {
     PORTB &= 0B11011111;
  }
  ledState = !ledState;
}

