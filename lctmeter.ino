// Pin Definitions
const byte PIN_INTR0 = 2;      // Interrupt 0 pin
const byte PIN_INTR1 = 3;      // Interrupt 1 pin
#define PIN_LED_TEMP 10         // Temp mode on/off LED
#define PIN_LED_LC 9          // LC mode toggle LED
#define PIN_FREQ_INPUT 5       // Frequency Input: Pin D5

#define PIN_RELAY1 11          // Relay 1: connects calibration cap in parallel
#define PIN_RELAY23 12         // Relay 2 & 3: controls inductor and capacitor

// Constants
#define CAPACITANCE_CALIBRATION 1e-9
#define NTC_PIN A7

#define NOMINAL_RESISTANCE 10000   // Nominal resistance at 25⁰C
#define NOMINAL_TEMP 25            // Temperature for nominal resistance (usually 25⁰C)
#define BETA_COEFFICIENT 3737      // Beta coefficient of the thermistor
#define REFERENCE_RESISTANCE 10000 // Resistor value in voltage divider

// Global Variables
volatile bool tempMode = false;
volatile bool lcMode = false;

volatile unsigned long timerCounts;
volatile bool counterReady;

unsigned long overflowCount;
unsigned int timerTicks;
unsigned int timerPeriod;
double frequencyValue;
double temperature = 30.0;
double inductance, capacitance;
double freq1 = 553516;
double freq2 = 381300;

#include "U8glib.h"
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE);  // I2C / TWI OLED display

// Function Prototypes
void drawTemperatureScreen();
void drawInductanceScreen();
void drawCapacitanceScreen();
void checkTempMode();
void checkLCMode();
void startCounting(unsigned int ms);
float measureFrequency();
void calculateTemperature();
void calculateInductance();
void calculateCapacitance();

// Setup function
void setup() {
  Serial.begin(115200); 

  // Pin configurations
  pinMode(PIN_INTR0, INPUT_PULLUP);
  pinMode(PIN_INTR1, INPUT_PULLUP);
  pinMode(PIN_RELAY1, OUTPUT);
  pinMode(PIN_RELAY23, OUTPUT);
  pinMode(PIN_LED_TEMP, OUTPUT);
  pinMode(PIN_LED_LC, OUTPUT);
  digitalWrite(PIN_RELAY1, LOW);

  // Attach Interrupts
  attachInterrupt(digitalPinToInterrupt(PIN_INTR0), checkLCMode, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_INTR1), checkTempMode, RISING);
  }

// Main loop
void loop() {
  while (!tempMode) {
    if (!lcMode) {
      calculateInductance();
      if (tempMode) break;
    } else {
      calculateCapacitance();
      if (tempMode) break;
    }
  }

  while (tempMode) {
    calculateTemperature();
    }
  }

// Interrupt Handlers
void checkTempMode() {
  tempMode = !tempMode;
  digitalWrite(PIN_LED_TEMP, tempMode);
  }

void checkLCMode() {
  lcMode = !lcMode;
  digitalWrite(PIN_LED_LC, lcMode);
  }

// Counting and Frequency Measurement
void startCounting(unsigned int ms) {
  counterReady = false;
  timerPeriod = ms;
  timerTicks = 0;
  overflowCount = 0;

  // Reset Timer 1 and Timer 2
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR2A = 0;
  TCCR2B = 0;

  // Timer 1 - counts events on pin D5
  TIMSK1 = bit(TOIE1);  // interrupt on Timer 1 overflow

  // Timer 2 - gives us our 1 ms counting interval
  TCCR2A = bit(WGM21);  // CTC mode
  OCR2A  = 124;         // count up to 125 (zero-relative)
  TIMSK2 = bit(OCIE2A); // enable Timer2 Interrupt

  TCNT1 = 0;  // Reset counters
  TCNT2 = 0;     

  // Reset prescalers
  GTCCR = bit(PSRASY);  // reset prescaler

  // Start Timer 2
  TCCR2B = bit(CS20) | bit(CS22);  // prescaler of 128

  // Start Timer 1 (External clock source on T1 pin D5)
  TCCR1B = bit(CS10) | bit(CS11) | bit(CS12);
  }

ISR (TIMER1_OVF_vect) {
  ++overflowCount;  // count Timer 1 overflows
  }

ISR (TIMER2_COMPA_vect) {
  unsigned int timer1CounterValue = TCNT1;  // get counter value
  unsigned long overflowCopy = overflowCount;

  // Check if timing period reached
  if (++timerTicks < timerPeriod) return;

  // Handle missed overflow
  if ((TIFR1 & bit(TOV1)) && timer1CounterValue < 256) overflowCopy++;

  // Stop timers
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR2A = 0;
  TCCR2B = 0;

  // Disable interrupts
  TIMSK1 = 0;
  TIMSK2 = 0;

  // Calculate total count
  timerCounts = (overflowCopy << 16) + timer1CounterValue;
  counterReady = true;
  }

float measureFrequency() {
  byte oldTCCR0A = TCCR0A;
  byte oldTCCR0B = TCCR0B;
  TCCR0A = 0;
  TCCR0B = 0;

  startCounting(500);  // count for 500 ms

  while (!counterReady) { }

  frequencyValue = (timerCounts * 1000.0) / timerPeriod;

  TCCR0A = oldTCCR0A;
  TCCR0B = oldTCCR0B;

  return frequencyValue;
  }

// Temperature Calculation
void calculateTemperature() {
  float raw = analogRead(NTC_PIN);
  raw = 1023 / raw - 1;
  raw = REFERENCE_RESISTANCE / raw;

  float temp = raw / NOMINAL_RESISTANCE;
  temp = log(temp);
  temp /= BETA_COEFFICIENT;
  temp += 1.0 / (NOMINAL_TEMP + 273.15);
  temp = 1.0 / temp;
  temp -= 273.15;

  temperature = temp;

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");

  u8g.firstPage();
  do {
    drawTemperatureScreen();
    } while (u8g.nextPage());
  }

// Inductance Calculation
void calculateInductance() {
  digitalWrite(PIN_RELAY1, LOW);
  digitalWrite(PIN_RELAY23, LOW);
  double freq4 = measureFrequency();
  Serial.print(freq4);
  double f14=(freq1/freq4);
  double f12=(freq1/freq2);
  double div=1.21e7;//39.4784*freq1*freq1;
  inductance=((f14*f14-1)*(f12*f12-1)/div)/CAPACITANCE_CALIBRATION;
  //inductance = ((1 / (39.47 * freq4 * freq4 * CAPACITANCE_CALIBRATION)) - 86e-6) * 1e6;

  Serial.print("  Inductance: ");
  Serial.print(inductance);
  Serial.println(" uH");

  u8g.firstPage();
  do {
    drawInductanceScreen();
    } while (u8g.nextPage());
  }

// Capacitance Calculation
void calculateCapacitance() {
  digitalWrite(PIN_RELAY1, LOW);
  digitalWrite(PIN_RELAY23, HIGH);
  delay(500);  // Allow relay to stabilize
  //noInterrupts();
  double freq3 = measureFrequency();
  //interrupts();
  //double freq3 = measureFrequency();
  double fr13=freq1/freq3;
  Serial.print("FR13=");
  Serial.print(fr13);
  double fr12=freq1/freq2;
  Serial.print("  FR12=");
  Serial.print(fr12);
  capacitance=((fr13*fr13-1)/(fr12*fr12-1))*1e-3;
  Serial.print("  Frequency: ");
  Serial.print(freq3);
  //capacitance = ((1/(39.47 * frequencyValue * frequencyValue * 86e-6)) - 1e-9) * 1e6;

  Serial.print("  Capacitance: ");
  Serial.println(capacitance);

  u8g.firstPage();
  do {
    drawCapacitanceScreen();
    } while (u8g.nextPage());
  }

// Display Functions
void drawTemperatureScreen() {
  u8g.setFont(u8g_font_unifont);
  u8g.setPrintPos(20, 10);
  u8g.print("Temperature");
  u8g.setPrintPos(5, 20);
  u8g.setScale2x2();
  u8g.print(temperature);
  u8g.write(176);
  u8g.print("C");
  u8g.undoScale();
  u8g.setPrintPos(50, 60);
  u8g.print(temperature * 1.8 + 32);
  u8g.write(176);
  u8g.print("F");
  }

void drawInductanceScreen() {
  u8g.setFont(u8g_font_unifont);
  u8g.setPrintPos(20, 10);
  u8g.print("Inductance");
  u8g.setPrintPos(5, 20);
  u8g.setScale2x2();
  u8g.print(inductance);
  u8g.setPrintPos(30, 30);
  u8g.write(181);
  u8g.print("H");
  u8g.undoScale();
  }

void drawCapacitanceScreen() {
  u8g.setFont(u8g_font_unifont);
  u8g.setPrintPos(20, 10);
  u8g.print("Capacitance");
  u8g.setPrintPos(05, 20);
  u8g.setScale2x2();        //double font size
  //if(capacitance<1e-9)
  u8g.print(capacitance);
  //u8g.write(176);         //ASCII code of degree symbol is 176
  u8g.setPrintPos(30, 30);
  u8g.write(181);
  u8g.print("F");
  u8g.undoScale();
  }
