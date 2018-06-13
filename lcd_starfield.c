/*
 * Graphics 128x64 LCD (Samsung KS0108 controller)
 *
 * Connections:
 * D/nI - Arduino pin 8 (ATmega 328P: PB0)
 * R/nW - Arduino pin 9 (ATmega 328P: PB1)
 * E   - Arduino pin 10 (ATmega 328P: PB2)
 * DB0-DB7 - Arduino pins 0-7 (ATmega 328P: PD0-PD7)
 * CS1 & CS2 - Arduino pins 11 and 12 (ATmega 328P: PB3, PB4)
 * nRST - Arduino pin 13 (ATmega 328P: PB5)
 *
 * Created: 03.02.2018
 * Author : acg
 */ 
#include <avr/io.h>
#include <avr/interrupt.h> // Needed for cli() and sei()

#define F_CPU 16000000L

// By default, sleep duration is 16 timer ticks = 1 us
#define STD_SLEEP_TICKS 16

// Bit mask used to set CS1 in a control byte
#define PCSMASK1 0B00001000

// Bit mask used to set CS2 in a control byte
#define PCSMASK2 0B00010000

// Stores the screen data: every 8 vertical pixes are
// a byte stored as a value in the buffer
volatile uint8_t buffer[1024];

// The current coordinates of the stars:
// stars[i] = 128 -> no star on row i
// stars[i] = k -> star on row i is at column k
volatile uint16_t starpos[64];

// The current speeds of the stars:
// speeds[i] = 0 -> no star on row i
// speeds[i] = k -> star on row i has speed k in [1, 7]
volatile uint16_t starspeed[64];

// The CRC polynomial used in generating random numbers
volatile uint16_t pol = 0xc0de;

// The current seed used in the CRC random number computations
volatile uint16_t currentSeed = 0x0003;

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
* Random number generator related functions
*************************************************/

/*
 * Generates the next random number given the globally defined
 * polynomial pol and the seed value currentSeed 
 * @return 16 bit random value
 */
uint16_t getNextRandom() {
  uint8_t xorFlag = 0;
  // Test LSB
  if ((currentSeed & 0x0001) == 1) {
    xorFlag = 1;
  }
  currentSeed = currentSeed >> 1;
  if (xorFlag) {
    currentSeed = currentSeed ^ pol;
  }  
  return currentSeed;
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
 * Uses the random number generator to initialise the screen at t=0:
 * - It initialises starpos[64] with 128 for rows without stars and 0 for
 * rows with stars.
 * - It initialises starspeed[64] with 0 for rows that have no star and
 * a value between 1 and 7 for rows with stars.
 * The screen is split in 4 zones (=16 pixel high slice of screen).
 */
void initialiseStars() {
  uint16_t rand16seed1 = 0x0007;
  uint16_t rand16seed2 = 0x0003;
  uint8_t zone = 0;
  uint8_t baseZoneY = 0;
  while (zone < 4) {
    baseZoneY = 16 * zone;
    // Get the star positions for this zone -- we do not init each star
    // manually in order to get a bit of randomness in how many stars we
    // get per zone
    rand16seed1 = getNextRandom();
    for (uint8_t i = 0; i < 16; i++) {      
      if (rand16seed1 & (0x0001 << i)) {
        starpos[baseZoneY + i] = 0;
        rand16seed2 = getNextRandom();
        starspeed[baseZoneY + i] = rand16seed2 & 0x0007;
      }
      else {
        starpos[baseZoneY + i] = 128;
        starspeed[baseZoneY + i] = 0;
      }
    }      
    zone++;
  }
}

/*
 * Generates a position for a new star: this is the line
 * on which the star will reside.
 * @return The position value, between 0 and 63
 */
uint16_t generateNewPos() {
  uint16_t nextRandom = getNextRandom();
  // We keep generating random line number as long as
  // the ones we generate are occupied (=their speed is not 0)
  // 0x003F -> we want to limit the values to the interval [0, 63]
  while (starspeed[nextRandom & 0x003F] != 0) {
    currentSeed = nextRandom;
    nextRandom = getNextRandom();
  }
  return nextRandom & 0x003F;
}

/*
 * Generates a speed for a new star, using the currently set seed
 * "left over" from the previous computation
 * @return The speed value (=3 LSB of the generated 16 bit value)
 */
uint16_t generateNewSpeed() {
  uint16_t nextRandom = getNextRandom();
  return nextRandom & 0x0007;
}

/*
 * Updates the star positions based on their current positions and relative speeds.
 */
void updateStarPositions() {
  // At every tick, move star i a number of dots equal to starspeed[i]
  for (uint8_t i = 0; i < 64; i++) {
    // If the speed is 0, there's no star on line i
    if (starspeed[i] != 0) {
      // Clear the currently occupied position for star i
      clearPixel(starpos[i], i);
      // Compute the potential new position of the star
      uint16_t newpos = starpos[i] + starspeed[i];
      // If the move would make the star fall off the screen
      if (newpos >= 128) {
        // Generate a new star on a random (empty) line
        uint8_t newLineIndex = generateNewPos();
        starpos[newLineIndex] = 0;
        // And generate also a random speed for the new star
        starspeed[newLineIndex] = generateNewSpeed();
        // Making sure that we don't have a speed equal to 0
        starspeed[newLineIndex] = (starspeed[newLineIndex] == 0) ? 1 : starspeed[newLineIndex];
        // Update the position in the screen buffer
        setPixel(0, newLineIndex);
        // Make the old star fall off the screen
        starpos[i] = 128;
        starspeed[i] = 0;
      } else {
        // No falling off the screen yet, so just update the position
        starpos[i] = newpos;
        setPixel(starpos[i], i);
      }
    }
  }
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

int main(void) {
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
  initialiseStars();
  while (1) {
    if (displayNowFlag) {
      // Set the flag to false
      displayNowFlag = 0;
      // Send precomputed frame to LCD
      updateScreen();
      // Precompute next frame
      updateStarPositions();
    }
  }
}

// Invoked every 100ms, sets the displayNowFlag
ISR(TIMER1_COMPA_vect) {
  displayNowFlag = 1;
}