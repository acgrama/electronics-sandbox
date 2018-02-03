/*
 * Control servo with joystick:
 * - Servo is connected on Arduino digital pin 9 (ATmega 328P pin 15)
 * - Joystick is connected to Arduino analog pin 0 (ATmega 328P pin 23, ADC0)
 *
 * Created: 28.01.2018
 * Author : acg
 */ 
#include <avr/io.h>
#include <avr/interrupt.h> // Needed for cli() and sei()

#define F_CPU 16000000L
#define PWM_TOP 39999;

volatile uint32_t adcValue;

/*
  Sets up PWM for pin 15 (PORTB1) / Arduino pin 9                    
  @param dutyCount The new compare value, corresponding to
  the new duty cycle
*/
void initPWMTimer(uint16_t dutyCount) {
  // We use Timer 1
  // Set the TOP value s.t. f_pwm = 50Hz
  // f_pwm = f_clk / (N * (1 + TOP))
  ICR1 = PWM_TOP; // TOP -- this will not change

  // Set the compare value to the new one
  OCR1A = dutyCount;
    
  // Set COM1A1 = 1, COM1B1 = 1
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1);

  // Set WGM3:0 = B1110
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << WGM12);
    
  // Set the prescaler to 8
  TCCR1B |= (1 << CS11);
}

/*
  Sets up ADC in free running mode
 */
void setupADC() {
  // 0. Disable digital input buffer on ADC0
  DIDR0 |= (1 << ADC0D);
  
  // 1. Select the channel
  // We use ADC0, which is MUX[3:0]=0000
  // 2. Select the voltage reference
  // We use AREF, which is 00
  ADMUX = 0B00000000;
  
  // 3. Set the ADC prescaler to 64 ADPS[2:0]=110
  ADCSRA = (1 << ADPS2) | (1 << ADPS1);
  
  // 4. Set it to free running mode (0000)
  ADCSRB = 0B00000000;
  
  // 5. Set the ADIE, for the ADC ISR
  ADCSRA |= (1 << ADIE);
  
  // 6. Set the auto trigger option on
  ADCSRA |= (1 << ADATE);
  
  // 7. Enable ADC
  ADCSRA |= (1 << ADEN);
  
  // 8. Start conversion
  ADCSRA |= (1 << ADSC);
}  

int main(void) {
  // Set port C0 as input (ADC)
  DDRC |= 0B00000000;
  // Set port B1 as output (LED)
  DDRB |= 0B00000010;
  cli();
  setupADC();
  sei();
  
  while (1) {
    // Voltage level on the joystick input (0 - 1023) is continuously
    // written into the volatile adcVoltage variable
    
    // We map the [0, 1023] ADC interval to the [2000, 4000] PWM interval
    // (which corresponds to [5%, 10%] servo PWM duty cycle)
    
    // First, we map to the equivalent [0, 2000] interval
    uint16_t dutyTop = 2000;
    uint32_t pwmValue = adcValue * dutyTop;
    // Divide by the max adc value 1023 (~2^10)
    pwmValue = (pwmValue >> 10);
    
    // Add back the 2000 we cut to align the PWM interval to 0
    pwmValue += 2000;
    
    // Set the PWM timer's compare value to the corresponding percent of TOP value
    initPWMTimer((uint16_t)pwmValue);
  }
}

/*
  The ISR which is invoked when an ADC conversion is finished
 */
ISR(ADC_vect) {
  // Copy the converted value into our volatile variable
  adcValue = (uint32_t)ADC;
}