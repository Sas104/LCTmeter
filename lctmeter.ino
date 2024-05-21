const byte intr0 = 2;  //interrupt 0 pin
const byte intr1 = 3;  //interrupt 1 pin
#define LEDT 9     //temp mode on/off
#define LEDLC 10   //Lor c mode toggle

// Frequency Input: Pin D5

#define relay1 11  //connects calibration cap in parallel
//relay 2 connects inductor in series when OFF
//relay 2 connects capacitor in parallel when ON
//relay 3 connects L1 terminal to GND when ON 
#define relay23 12 //turms on and off relay 2 & 3 at a time

// Boolean to represent toggle state
volatile bool tempmode = false;
volatile bool lcmode = false;  //inductor when false; capacitor when true

// these are checked for in the main program
volatile unsigned long timerCounts;
volatile boolean counterReady;

// internal to counting routine
unsigned long overflowCount;
unsigned int timerTicks;
unsigned int timerPeriod;
double frq, freq2=382000, freq3, freq4;
double freq1=544300;  //measured frequency when any external connection is absent. 
#define ccal 1e-9

#include "U8glib.h"
#define ntc A7
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE);  // I2C / TWI
double temperature=30, inductance, capacitance;

#define nominal_resistance 10000  //Nominal resistance at 25⁰C
#define nominal_temp 25   // temperature for nominal resistance (almost always 25⁰ C)
#define beta 3737     // The beta coefficient or the B value of the thermistor (usually 3000-4000) check the datasheet for the accurate value.
#define Rref 10000   //Value of  resistor used for the voltage divider

void drawtemp(void) {
  // graphic commands to redraw the complete screen should be placed here
  u8g.setFont(u8g_font_unifont);
  u8g.setPrintPos(20, 10);
  u8g.print("Temperature");
  u8g.setPrintPos(05, 20);
  u8g.setScale2x2();        //double font size
  u8g.print(temperature);
  u8g.write(176);         //ASCII code of degree symbol is 176
  u8g.print("C");
  u8g.undoScale();       //undo font size
  u8g.setPrintPos(50, 60);
  u8g.print(temperature*1.8+32);  // convert into farenheit. 
  u8g.write(176);      //ASCII code of degree symbol is 176
  u8g.print("F");
}
void drawind(void) {
  // graphic commands to redraw the complete screen should be placed here
  u8g.setFont(u8g_font_unifont);
  u8g.setPrintPos(20, 10);
  u8g.print("Inductance");
  u8g.setPrintPos(05, 20);
  u8g.setScale2x2();        //double font size
  u8g.print(inductance);
  //u8g.write(176);         //ASCII code of degree symbol is 176
  u8g.setPrintPos(30, 30);
  u8g.write(181);
  u8g.print("H");
  u8g.undoScale();       //undo font size
}
void drawcap(void) {
  // graphic commands to redraw the complete screen should be placed here
  u8g.setFont(u8g_font_unifont);
  u8g.setPrintPos(20, 10);
  u8g.print("Capacitance");
  u8g.setPrintPos(05, 20);
  u8g.setScale2x2();        //double font size
  //if(capacitance<1e-9)
  u8g.print(capacitance);
  //u8g.write(176);         //ASCII code of degree symbol is 176
  u8g.print("F");
  u8g.undoScale();       //undo font size
}
void checktmode(){
  // Check status of switch
  // Toggle LED if button pressed
 
  //if (digitalRead(intr1) == LOW) {
    // Switch was pressed
    // Change state of toggle
    tempmode = !tempmode;
    // Indicate state on LED
    digitalWrite(LEDT, tempmode);
  //}
}

void checklcmode(){
  // Check status of switch
  // Toggle LED if button pressed
  //if (digitalRead(intr0) == LOW) {
    // Switch was pressed
    // Change state of toggle
    lcmode = !lcmode;
    // Indicate state on LED
    digitalWrite(LEDLC, lcmode);
  //}
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); 
  pinMode(intr0, INPUT_PULLUP);
  pinMode(intr1, INPUT_PULLUP);
  pinMode(relay1, OUTPUT);
  pinMode(relay23, OUTPUT);
  pinMode(LEDT, OUTPUT);
  pinMode(LEDLC, OUTPUT);
  // Attach Interrupt to Interrupt Service Routine
  attachInterrupt(digitalPinToInterrupt(intr0),checklcmode, RISING); 
  attachInterrupt(digitalPinToInterrupt(intr1),checktmode, RISING); 
}

void loop() {
  // put your main code here, to run repeatedly:
  while(tempmode==false){
    while(lcmode==false){
      //measure inductance and display
      //Serial.println("Measuring inductor");
      //delay(200);
      calcind();
      if(tempmode){
        break;
      }
    }
    while(lcmode){
      //measure capacitance and display
      // Serial.println("Measuring capacitor");
      // delay(200);
      calccap();
            if(tempmode){
        break;
      }
    }
  }
  while(tempmode){
    //measure temperature and display
    Serial.println("Measuring temperature");
    calcTemp();
    //delay(200);
  }
}

void startCounting (unsigned int ms) 
  {
  counterReady = false;         // time not up yet
  timerPeriod = ms;             // how many 1 ms counts to do
  timerTicks = 0;               // reset interrupt counter
  overflowCount = 0;            // no overflows yet
  // reset Timer 1 and Timer 2
  TCCR1A = 0;             
  TCCR1B = 0;              
  TCCR2A = 0;
  TCCR2B = 0;
  // Timer 1 - counts events on pin D5
  TIMSK1 = bit (TOIE1);   // interrupt on Timer 1 overflow
  // Timer 2 - gives us our 1 ms counting interval
  // 16 MHz clock (62.5 ns per tick) - prescaled by 128
  //  counter increments every 8 µs. 
  // So we count 125 of them, giving exactly 1000 µs (1 ms)
  TCCR2A = bit (WGM21) ;   // CTC mode
  OCR2A  = 124;            // count up to 125  (zero relative!!!!)
  // Timer 2 - interrupt on match (ie. every 1 ms)
  TIMSK2 = bit (OCIE2A);   // enable Timer2 Interrupt
  TCNT1 = 0;      // Both counters to zero
  TCNT2 = 0;     
  // Reset prescalers
  GTCCR = bit (PSRASY);        // reset prescaler now
  // start Timer 2
  TCCR2B =  bit (CS20) | bit (CS22) ;  // prescaler of 128
  // start Timer 1
  // External clock source on T1 pin (D5). Clock on rising edge.
  TCCR1B =  bit (CS10) | bit (CS11) | bit (CS12);
  }  // end of startCounting

ISR (TIMER1_OVF_vect)
  {
  ++overflowCount;               // count number of Counter1 overflows  
  }  // end of TIMER1_OVF_vect

//******************************************************************
//  Timer2 Interrupt Service is invoked by hardware Timer 2 every 1 ms = 1000 Hz
//  16Mhz / 128 / 125 = 1000 Hz

ISR (TIMER2_COMPA_vect) 
  {
  // grab counter value before it changes any more
  unsigned int timer1CounterValue;
  timer1CounterValue = TCNT1;  // see datasheet, page 117 (accessing 16-bit registers)
  unsigned long overflowCopy = overflowCount;

  // see if we have reached timing period
  if (++timerTicks < timerPeriod) 
    return;  // not yet

  // if just missed an overflow
  if ((TIFR1 & bit (TOV1)) && timer1CounterValue < 256)
    overflowCopy++;

  // end of gate time, measurement ready

  TCCR1A = 0;    // stop timer 1
  TCCR1B = 0;    

  TCCR2A = 0;    // stop timer 2
  TCCR2B = 0;    

  TIMSK1 = 0;    // disable Timer1 Interrupt
  TIMSK2 = 0;    // disable Timer2 Interrupt

  // calculate total count
  timerCounts = (overflowCopy << 16) + timer1CounterValue;  // each overflow is 65536 more
  counterReady = true;              // set global flag for end count period
  }  // end of TIMER2_COMPA_vect

//Frequency counting function
float frequency(){
  // stop Timer 0 interrupts from throwing the count out
  byte oldTCCR0A = TCCR0A;
  byte oldTCCR0B = TCCR0B;
  TCCR0A = 0;    // stop timer 0
  TCCR0B = 0;    

  startCounting (500);  // how many ms to count for

  while (!counterReady) 
     { }  // loop until count over

  // adjust counts by counting interval to give frequency in Hz
  frq = (timerCounts *  1000.0) / timerPeriod;

  // restart timer 0
  TCCR0A = oldTCCR0A;
  TCCR0B = oldTCCR0B;

  return frq;
}

void calcTemp(){
  float raw = analogRead(ntc);
  raw=1023/raw-1;     //this equals R/Rref
  raw=Rref/raw;       //Rref^2/R
  float temp;
  temp = raw/ nominal_resistance;     // (R/Ro)
  temp = log(temp);                  // ln(R/Ro)
  temp /= beta;                   // 1/B * ln(R/Ro)
  temp += 1.0 / (nominal_temp + 273.15); // + (1/To)
  temp = 1.0 / temp;                 // Invert
  temp -= 273.15;                         // convert absolute temp to C
  temperature=temp;
  // Print the temperature value to the serial monitor
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");
  // picture loop
  u8g.firstPage();
  do {
    drawtemp();
  } while (u8g.nextPage());
  // rebuild the picture after some delay
  //delay(500);
}

void calcind(){

  digitalWrite(relay23, LOW);
  freq4=frequency();
  inductance=((1/(39.47*freq4*freq4*1e-9))-86e-6)*1e6;
  Serial.print("Inductance:");
  Serial.print(inductance);
  Serial.println("uH");
  //4*(pi)^2 is 39.4784176
  // picture loop
  u8g.firstPage();
  do {
    drawind();
  } while (u8g.nextPage());
  // rebuild the picture after another cycle
}
void calccap(){
  digitalWrite(relay23, HIGH);  //turn on the connection for cdet
  delay(500);  //relay require 20ms to turn on. 10 times delay for sutained oscillation
  freq3=frequency();
  Serial.println(freq3);
  capacitance=((1/(39.47*freq3*freq3*86e-6))-1e-9)*1e6;
  Serial.print("cap: ");
  Serial.println(capacitance);
  // picture loop
  u8g.firstPage();
  do {
    drawcap();
  } while (u8g.nextPage());
  // rebuild the picture after another cycle
}
