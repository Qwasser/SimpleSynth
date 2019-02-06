#include <Arduino.h>
#include "tones.h"

#define maxWaveform 4
#define maxSamplesNum 120

#define NUMBER_OF_SHIFT_CHIPS   2
#define DATA_WIDTH   NUMBER_OF_SHIFT_CHIPS * 8
#define PULSE_WIDTH_USEC   5

byte rows[] = {43, 45, 47, 49, 51};

static unsigned int toneMatrix[5][8] = {
  {NOTE_B4, NOTE_C5, NOTE_AS4, NOTE_CS5, NOTE_A4, NOTE_D5, NOTE_GS4, NOTE_DS5},
  {NOTE_DS4, NOTE_E4, NOTE_D4, NOTE_F4, NOTE_CS4, NOTE_FS4, NOTE_C4, NOTE_G4},
  {NOTE_G3, NOTE_GS3, NOTE_FS3, NOTE_A3, NOTE_F3, NOTE_AS3, NOTE_E3, NOTE_B3},
  {NOTE_B2, NOTE_C3, NOTE_AS2, NOTE_CS3, NOTE_A2, NOTE_D3, NOTE_GS2, NOTE_DS3},
  {NOTE_DS2, NOTE_E2, NOTE_D2, NOTE_F2, NOTE_CS2, NOTE_FS2, NOTE_C2, NOTE_G2}
};

static volatile unsigned int currentMaxCounter = 0;
static volatile unsigned int currentCounter = 0;
static volatile unsigned int currentMaxCounter2 = 0;
static volatile unsigned int currentCounter2 = 0;
static volatile unsigned int currentMaxCounter3 = 0;
static volatile unsigned int currentCounter3 = 0;
static volatile unsigned int currentMaxCounter4 = 0;
static volatile unsigned int currentCounter4 = 0;
static unsigned int freqHz = 50000;

int ploadPin        = 8;  // Connects to Parallel load pin the 165
int dataPin         = 11; // Connects to the Q7 pin the 165
int clockPin        = 12; // Connects to the Clock pin the 165

void tone_matrix_to_counters() {
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 8; ++j) {
      toneMatrix[i][j] = freqHz / toneMatrix[i][j];
    }
  }
}

uint16_t read_shift_regs()
{
    long bitVal;
    uint16_t bytesVal = 0;

    /* Trigger a parallel Load to latch the state of the data lines,
    */
    digitalWrite(ploadPin, LOW);
    delayMicroseconds(PULSE_WIDTH_USEC);
    digitalWrite(ploadPin, HIGH);

    /* Loop to read each bit value from the serial out line
     * of the SN74HC165N.
    */
    for(int i = 0; i < DATA_WIDTH; i++)
    {
        bitVal = digitalRead(dataPin);

        /* Set the corresponding bit in bytesVal.
        */
        bytesVal |= (bitVal << ((DATA_WIDTH-1) - i));

        /* Pulse the Clock (rising edge shifts the next bit).
        */
        digitalWrite(clockPin, HIGH);
        delayMicroseconds(PULSE_WIDTH_USEC);
        digitalWrite(clockPin, LOW);
    }

    return(bytesVal);
}

void display_pin_values(uint16_t pinValues)
{
    SerialUSB.print("Pin States:\r\n");

    for(int i = 0; i < DATA_WIDTH; i++)
    {
        SerialUSB.print("  Pin-");
        SerialUSB.print(i);
        SerialUSB.print(": ");

        if((pinValues >> i) & 1)
            SerialUSB.print("HIGH");
        else
            SerialUSB.print("LOW");

        SerialUSB.print("\r\n");
    }

    SerialUSB.print("\r\n");
}

void read_current_tone() {
  int foundNotes = 0;
  for (int colIndex = 0; colIndex < 8; ++colIndex) {
    pinMode(colIndex + 34, OUTPUT);
    digitalWrite(colIndex + 34, LOW);
    for (int rowIndex = 0; rowIndex < 5; ++rowIndex) {
        byte rowPin = rows[rowIndex];
        pinMode(rowPin, INPUT_PULLUP);
        if (!digitalRead(rowPin)) {
          foundNotes += 1;

          switch (foundNotes)
          {
            case 1:
            currentMaxCounter = toneMatrix[rowIndex][colIndex];
            break;
            case 2:
            currentMaxCounter2 = toneMatrix[rowIndex][colIndex];
            break;
            case 3:
            currentMaxCounter3 = toneMatrix[rowIndex][colIndex];
            break;
            case 4:
            currentMaxCounter4 = toneMatrix[rowIndex][colIndex];
            break;
          }
        }
        pinMode(rowPin, INPUT);
    }
    pinMode(colIndex + 34, INPUT_PULLUP);
  }

  switch (foundNotes)
  {
    case 0:
    currentMaxCounter = 0;
    case 1:
    currentMaxCounter2 = 0;
    case 2:
    currentMaxCounter3 = 0;
    case 3:
    currentMaxCounter4 = 0;
  }
}

void play_current_tone() {
  int dac_val = 0;

  if (currentCounter > currentMaxCounter) {
    currentCounter = 0;
  }
  if (currentCounter > currentMaxCounter / 2) {
    dac_val += 0xff;
  }
  currentCounter += 1;

  if (currentCounter2 > currentMaxCounter2) {
    currentCounter2 = 0;
  }
  if (currentCounter2 > currentMaxCounter2 / 2) {
    dac_val += 0xff;
  }
  currentCounter2 += 1;

  if (currentCounter3 > currentMaxCounter3) {
    currentCounter3 = 0;
  }
  if (currentCounter3 > currentMaxCounter3 / 2) {
    dac_val += 0xff;
  }
  currentCounter3 += 1;

  if (currentCounter4 > currentMaxCounter4) {
    currentCounter4 = 0;
  }
  if (currentCounter4 > currentMaxCounter4 / 2) {
    dac_val += 0xff;
  }
  currentCounter4 += 1;

  analogWrite(DAC0, dac_val);
}

void TC3_Handler()
{
  TC_GetStatus(TC1, 0); // Set correct ticks
  play_current_tone();
}

void TimerStart(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t freq)
{
   pmc_set_writeprotect(false);
   pmc_enable_periph_clk(irq);
   TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
   uint32_t rc = VARIANT_MCK / 128 / freq;
   TC_SetRA(tc, channel, rc >> 1); // 50% duty cycle square wave
   TC_SetRC(tc, channel, rc);
   TC_Start(tc, channel);
   tc->TC_CHANNEL[channel].TC_IER=  TC_IER_CPCS | TC_IER_CPAS;
   tc->TC_CHANNEL[channel].TC_IDR=~(TC_IER_CPCS | TC_IER_CPAS);
   NVIC_EnableIRQ(irq);
}

void setup() {
  analogWriteResolution(12);
  pinMode(LED_BUILTIN, OUTPUT);
  SerialUSB.begin(9600);

  pinMode(ploadPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, INPUT);

  digitalWrite(clockPin, LOW);
  digitalWrite(ploadPin, HIGH);

  pinMode(43, INPUT);
  pinMode(45, INPUT);
  pinMode(47, INPUT);
  pinMode(49, INPUT);
  pinMode(51, INPUT);

  for (int i = 34; i <= 41; i++) {
    pinMode(i, INPUT_PULLUP);
  }
  
  tone_matrix_to_counters();
  
  TimerStart(TC1, 0, TC3_IRQn, freqHz);
}

void loop() {
  delayMicroseconds(25000);
  read_current_tone();
  // SerialUSB.println(currentNote);
}
