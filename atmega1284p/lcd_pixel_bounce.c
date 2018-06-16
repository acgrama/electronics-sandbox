/*
 * Pixel bouncing demo on graphics LCS 128x64 (KS0108 controller)
 * with ATmega1284P
 *
 * ---------------------
 * Connections
 * Data pins:
 * - DB0:DB7 on PORTD0:7
 *
 * Control pins:
 * - D/nI PORTB0
 * - R/nW PORTB1
 * - E PORTB2
 * - CS1 PORTB3
 * - CS2 PORTB4
 * - nRST PORTB5
 * ---------------------
 *
 * Created: 16.06.2018
 * Author : Cristina Einramhof-Grama
 */ 

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>

/* --- Global variables, e.g. for interrupt service routines --- */
volatile uint8_t ucPortCValue;

// By default, sleep duration is 16 timer ticks = 1 us
#define STD_SLEEP_TICKS 16

// Bit mask used to set CS1 in a control byte
#define PCSMASK1 0B00001000

// Bit mask used to set CS2 in a control byte
#define PCSMASK2 0B00010000

// Stores the screen data: every 8 vertical pixes are
// a byte stored as a value in the buffer
volatile uint8_t buffer[1024];

// The current coordinates of the pixel
volatile uint8_t posx, posy;

// The current deltas for each direction
// They change to -1 when their direction changes!
volatile int8_t deltax = 1, deltay = 1;

// Lookup table of all possible bitmasks with one bit
const uint8_t bitmasks[8] = { 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80 };

// Flag that is set to true when the period between two screen refreshes has passed
// If this is equal to 1, it means the time has come to refresh the screen with the updated screen buffer
volatile uint8_t displayNowFlag;

/*
 * Sets up TIMER0 for waveform generation delays:
 * - no prescaler
 * - normal mode of operation
 */
void timerSetup() {
  TCNT0 = 0B00000000;
  // COM0A1 = 0, COM0A0 = 0, COM0B1 = 0, COM0B0 = 0, WGM01 = 0, WGM00 = 0
  TCCR0A = 0B00000000;
  // FOC0A = 0, FOC0B = 0, WGM02 = 0, CS0[0:2] = 001
  TCCR0B = 0B00000001;
}

/*
 * Sets up TIMER1 for delays needed to keep the screen display non-jittery
 * - prescaler value = 256
 * - CTC Mode (Clear Timer on Compare Match)
 * - we want an interrupt generated every time we reach the TOP count value
 */
void displayTimerSetup() {
  // Initialise the Timer 1 registers
  // COM1A1 = 0, COM1A0 = 0, COM1B1 = 0, COM1B0 = 0, 0, 0, WGM11 = 0, WGM10 = 0
  TCCR1A = 0B00000000;
  
  // We use TIMER1 in CTC mode, so WGM1[2:0] = 100
  // Set prescaler value to 256: CS1[2:0] = 100
  // ICNC1 = 0, ICES1 = 0, 0, WGM13 = 0, WGM12 = 1, CS12 = 1, CS11 = 0, CS10 = 0
  TCCR1B = 0B00001100;
  
  // Enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  
  // Set compare match register to desired value for 10Hz frequency
  OCR1A = 6250; 
}

/*
 * Sleeps for a specified delay of time, given in timer ticks.
 * Since we use no prescaler, 1 timer tick = 1 T_clk = 1 / f_clk = 0.0625us
 * @param delay Number of timer ticks
 */
void sleepTicks(uint8_t delay) {
  uint8_t refTime = TCNT0;
  while ((uint8_t)(TCNT0 - refTime) < delay) { }
}

/*
 * Initial set-up:
 * - Sets port directions
 * - Disables UART (we are using Arduino pins 0 and 1, which are RX and TX, respectively)
 */
void setUp() {
  // Set PORTB as output
  DDRB = 0B11111111;
  // Disable fucking UART!!!
  UCSR0B = 0x00;
}

/*
 * Resets the LCD by setting the nRST = 0, waiting min tRS = 1us, setting nRST = 1
 * In practice, we wait a bit longer
 */
void resetLcd(void) {
  // nRST = 0 (PB5), CS2 = 0 (PB4), CS1 = 0 (PB3), E = 0 (PB2), R/nW = 0 (PB1), D/nI = 0 (PB0)
  PORTB = 0B00000000;
  sleepTicks(STD_SLEEP_TICKS * 10);
  // nRST = 1 (PB5), CS2 = 0 (PB4), CS1 = 0 (PB3), E = 0 (PB2), R/nW = 0 (PB1), D/nI = 0 (PB0)
  PORTB = 0B00100000;
  sleepTicks(STD_SLEEP_TICKS);
}

/*
 * Returns the appropriate bit mask, depending on the CS value
 * @param cs The value of the CS (1 or 2) 
 */
inline uint8_t getMask(uint8_t cs) {
  uint8_t PCSMASK = PCSMASK1;
  if (cs == 2) {
    PCSMASK = PCSMASK2;
  }
  return PCSMASK;
}

/*
 * Waits while the device is busy
 * @param cs The value of the CS (1 or 2) 
 */
void waitWhileBusy(uint8_t cs) {
  uint8_t PCSMASK = getMask(cs);
  // Set the mode for pins on PORTD to "input"
  DDRD = 0B00000000;
  // We want to send an instruction (read the status flag), for the selected chip
  // We start by assuming the chip is "very" busy (= PD7 BUSY=1 && PD4 RESET=1)
  // (actually, any value != 0x00 would do)
  uint8_t isBusy = 0x90;
  while (isBusy != 0x00) {
    PORTB = 0B00100010 | PCSMASK;
    sleepTicks(STD_SLEEP_TICKS);
    // Set E = 1 (PB2)
    PORTB = 0B00100110 | PCSMASK;
    sleepTicks(STD_SLEEP_TICKS);
    // Now, the data should be ready on PORTD, but check first whether it's busy or reset
    // isBusy = PORTD7;
    // isReset = PORTD4;
    isBusy = PIND & 0x90;    
    // Set E = 0 (PB2)
    PORTB = 0B00100010 | PCSMASK;
    sleepTicks(STD_SLEEP_TICKS);
  }
}

/*
 * Writes an instruction to the controller
 * @param cs The value of the CS (1 or 2)
 * @param data The data that is written as part of the instruction
 */
void writeInstruction(uint8_t cs, uint8_t data) {
  waitWhileBusy(cs);
  uint8_t PCSMASK = getMask(cs);
  // Set the mode for pins on PORTD to "output"
  DDRD = 0B11111111;
  PORTB = 0B00100000 | PCSMASK;
  sleepTicks(STD_SLEEP_TICKS);
  // Set E = 1 (PB2)
  PORTB = 0B00100100 | PCSMASK;
  PORTD = data;
  sleepTicks(STD_SLEEP_TICKS);
  // Set E = 0 (PB2)
  PORTB = 0B00100000 | PCSMASK;
  sleepTicks(STD_SLEEP_TICKS);
}

/*
 * Writes a byte of display data to the currently set (X+Y) address.
 * It outputs a byte on DB[0:7]
 *
 * @param cs The value of the CS (1 or 2)
 * @param data The data to be written on DB[0:7]
 */
void writeDisplayData(uint8_t cs, uint8_t data) {
  waitWhileBusy(cs);
  uint8_t PCSMASK = getMask(cs);
  // Set PORTD pins to "output"
  DDRD = 0B11111111;
  PORTB = 0B00100001 | PCSMASK;
  sleepTicks(STD_SLEEP_TICKS);
  // Set E = 1 (PB2)
  PORTB = 0B00100101 | PCSMASK;
  PORTD = data;
  sleepTicks(STD_SLEEP_TICKS);
  // Set E = 0 (PB2)
  PORTB = 0B00100001 | PCSMASK;
  sleepTicks(STD_SLEEP_TICKS);
}

/*************************************************
* Data display related functions
*************************************************/
/*
 * Clears the screen buffer
 */
void clearScreenBuffer() {
  for (uint16_t i = 0; i < sizeof(buffer); i++) {
    buffer[i] = 0;
  }
}

/*
 * Computes the bitmask that is equivalent to setting a pixel
 * at logical screen position (posX, posY)
 * The screen is considered to be in landscape mode, and the
 * (x, y) screen coordinates are mathematically coherent, i.e.
 * x is the longer side with values in [0, 127]
 * y is the shorter side with values in [0, 63]
 *
 * @param posY The pixel coordinate y
 */
uint8_t computeBitmask(uint8_t posY) {
  // The bitmask is equivalent to shifting one bit by the modulo y % 8
  // The modulo can be computed faster by extracting the 3 LSBs
  // from y (since modulo 2^n = n LSBs)
  return bitmasks[posY & 7];
}

/*
 * Computes the index at which the bitmask value has to be
 * inserted in the screen buffer.
 *
 * @param posX The pixel coordinate x
 * @param posY The pixel coordinate y
 * @return The 16 bit index value
 */
uint16_t computeBufferIndex(uint8_t posX, uint8_t posY) {
  // The index is (posY / 8) * 128 + posX
  // Div 8 is shr 3
  // Mul 128 is shl 7
  // Put together, we can do a simplified shl of 7-3=4
  // But first we have to AND out the 3 bits that would be
  // lost during the initial shr 3  
  return ((posY & 0B11111000) << 4) + posX;
}

/*
 * Sets a pixel value in the screen buffer
 *
 * @param posX The pixel coordinate x
 * @param posY The pixel coordinate y
 */
void setPixel(uint8_t posX, uint8_t posY) {
  uint8_t bitmask = computeBitmask(posY);
  uint16_t index = computeBufferIndex(posX, posY);
  buffer[index] = bitmask;
}

/*
 * Clears a pixel value in the screen buffer
 *
 * @param posX The pixel coordinate x
 * @param posY The pixel coordinate y
 */
void clearPixel(uint8_t posX, uint8_t posY) {
  uint16_t index = computeBufferIndex(posX, posY);
  buffer[index] = 0;
}

/*
 * Checks whether a position together with a direction of
 * movement signifies a "wall hit". For instance, going in
 * a southward direction and reaching y = 63 (bottom wall)
 * or x = 127 (right wall).
 *
 * @param x The pixel coordinate x
 * @param y The pixel coordinate y
 * @return 1 if it is a hit, 0 otherwise
 */
uint8_t isWallHit(uint8_t x, uint8_t y) {
  return ((deltax < 0) && (x == 0)) || ((deltax > 0) && (x == 127)) || ((deltay < 0) && (y == 0)) || ((deltay > 0) && (y == 63));
}

/*
 * Changes the direction, i.e. the deltax or deltay
 * @param x The pixel coordinate x
 * @param y The pixel coordinate y
 */
void changeDir(uint8_t x, uint8_t y) {
  if ((x == 0) || (x == 127))
    deltax = -deltax;
  if ((y == 0) || (y == 63))
    deltay = -deltay;
}

/*
 * Moves the pixel one position in the current direction.
 * Whoever invokes this function must guarantee that we are
 * not "falling off" the screen in doing so.
 */
void movePixel() {
  if (isWallHit(posx, posy))
    changeDir(posx, posy);
  posx += deltax;
  posy += deltay;
}

/*
 * Writes the contents of the screen buffer into screen memory
 */
void updateScreen() {
  uint8_t ramX = 0;
  uint8_t ramY = 0;
  while (ramX < 8) {
    // Set page address to ramX for both CS
    writeInstruction(1, 0B10111000 | ramX);
    writeInstruction(2, 0B10111000 | ramX);
    ramY = 0;
    // Set byte address to 0 for both CS
    // ramY will automatically increment when writing the data
    writeInstruction(1, 0B01000000);
    writeInstruction(2, 0B01000000);
    uint16_t base = (uint16_t)ramX * 128;
    while (ramY < 64) {
      writeDisplayData(1, buffer[base + ramY]);
      writeDisplayData(2, buffer[base + ramY + 64]);
      ramY++;
    }
    ramX++;
  }
}

/* --- Starting point of program execution after power-on reset --- */
int main(void)
{ 
  cli();
  // Set up the data transmission timer
  timerSetup();
  // Set up the timer used in controlling screen refresh rate
  displayTimerSetup();
  sei();
  setUp();
  sleepTicks(STD_SLEEP_TICKS);
  resetLcd();
  // Turn both chips on
  writeInstruction(1, 0B00111111);
  writeInstruction(2, 0B00111111);
  clearScreenBuffer();
  posx = 0;
  posy = 0;
  setPixel(posx, posy);
  while (1) {
    if (displayNowFlag) {
      // Set the flag to false
      displayNowFlag = 0;
      // Send precomputed frame to LCD
      updateScreen();
      // Precompute next frame
      clearPixel(posx, posy);
      movePixel();
      setPixel(posx, posy);
    }
  }
}

/* -------------- Interrupt Vectors -------------- */

/* --- Timer/Counter1 Compare Match A - Interrupt Vector #13 --- */
// Invoked every 100ms, sets the displayNowFlag
ISR(TIMER1_COMPA_vect) {
  displayNowFlag = 1;
}
