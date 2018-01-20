/************************************************************************/
/* Sets up PWM for pin 15 (PORTB1) / Arduino port 9                     
/* for a specific frequency (2Hz)
/************************************************************************/
#define F_CPU 16000000L

#include <avr/io.h>
#include <avr/interrupt.h> // Needed for cli() and sei()
#include <util/delay.h>

/************************************************************************/
/* Sets up PWM for pin 15 (PORTB1) / Arduino port 9                     */
/************************************************************************/
void initPWMTimer() {
  // Set port B1 as output
  DDRB |= 0B00000010;
  
  // We use Timer 1
  // Set the TOP value s.t. f_pwm = 2Hz
  // f_pwm = f_clk / (N * (1 + TOP))
  ICR1 = 7180; // TOP

  // Set the compare value to 25% of TOP
  OCR1A = 1500;
    
  // Set COM1A1 = 1, COM1B1 = 1
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1);

  // Set WGM3:0 = B1110
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << WGM12);
    
  // Set the prescaler to 1024
  TCCR1B |= (1 << CS12) | (1 << CS10);  
}

int main(void) {
  initPWMTimer();
  
  while (1) {
  }
}
