/**********************************************************
                Fast PWM Test
Demostrates the generation of high speed PWM
using timers 1 and 4
There are two pieces of code:
One for pins 9, 10 and 11 using TIMER 1
with frequencies up to 62kHz
Other for pins 6 and 13 using TIMER 4
with frequencies up to 187kHz
History:
  12/12/2014 - Version 1.0
  22/12/2014 - Adding a missing OCR4C value

  http://r6500.blogspot.com/2014/12/fast-pwm-on-arduino-leonardo.html
  
***********************************************************/

// https://web.archive.org/web/20140101032749/http://coolarduino.wordpress.com/2012/09/19/quasi-real-time-oscilloscope-remix/



/**********************************************************
   Fast PWM on pins 9,10,11 (TIMER 1)
   
   Do not use analogWrite to pins 9, 10 or 11 if using 
   this functions as they use the same timer.
   
   Those functions will probably conflict with the 
   servo library.
   
   Uses 5 PWM frequencies between 61Hz and 62.5kHz
   
**********************************************************/




/**********************************************************
   Fast PWM on pins 6, 13 (High Speed TIMER 4)
   
   Do not use analogWrite to pins 6 or 13 if using 
   this functions as they use the same timer.
   
   Those functions will conflict with the 
   MSTIMER2 library.
   Uses 7 PWM frequencies between 2930Hz and 187.5kHz
   
   Timer 4 uses a PLL as input so that its clock frequency
   can be up to 96MHz on standard Arduino Leonardo.
   We limit imput frequency to 48MHz to generate 187.5kHz PWM
   If needed, we can double that up to 375kHz
**********************************************************/

// Frequency modes for TIMER4
#define PWM187k 1   // 187500 Hz
#define PWM94k  2   //  93750 Hz
#define PWM47k  3   //  46875 Hz
#define PWM23k  4   //  23437 Hz
#define PWM12k  5   //  11719 Hz
#define PWM6k   6   //   5859 Hz
#define PWM3k   7   //   2930 Hz

// Direct PWM change variables
#define PWM6        OCR4D
#define PWM13       OCR4A

// Terminal count
#define PWM6_13_MAX OCR4C

// Configure the PWM clock
// The argument is one of the 7 previously defined modes
void pwm613configure(int mode)
{
// TCCR4A configuration
TCCR4A=0;

// TCCR4B configuration
TCCR4B=mode;

// TCCR4C configuration
TCCR4C=0;

// TCCR4D configuration
TCCR4D=0;

// TCCR4D configuration
TCCR4D=0;

// PLL Configuration
// Use 96MHz / 2 = 48MHz
// PLLFRQ=(PLLFRQ&0xCF)|0x30;
 PLLFRQ=(PLLFRQ&0xCF)|0x10; // Will double all frequencies

// Terminal count for Timer 4 PWM
OCR4C=255;
}

// Set PWM to D6 (Timer4 D)
// Argument is PWM between 0 and 255
void pwmSet6(int value)
{
OCR4D=value;   // Set PWM value
DDRD|=1<<7;    // Set Output Mode D7
TCCR4C|=0x09;  // Activate channel D
}

// Set PWM to D13 (Timer4 A)
// Argument is PWM between 0 and 255
void pwmSet13(int value)
{
OCR4A=value;   // Set PWM value
DDRC|=1<<7;    // Set Output Mode C7
TCCR4A=0x82;  // Activate channel A
}

/*************** ADDITIONAL DEFINITIONS ******************/

// Macro to converts from duty (0..100) to PWM (0..255)
#define DUTY2PWM(x)  ((255*(x))/100)

/**********************************************************/

////////////////////////////////////////
//////////        WHOA        //////////
//////////        WHOA        //////////
//////////        WHOA        //////////
//////////        WHOA        //////////
//////////        WHOA        //////////
////////////////////////////////////////

// Pins 

int hvDigitalIn = 19; // pf6 -> A1   D19
int hvClock = 18;     // pf7 -> A0   D18

int clearPin = 6;     // pd7 -> A7   D6#
int sense1 = A6;      // pd4 -> A6   D4
int sense2 = A11;     // pd6 -> A11  D12
int sense3 = 8;       // pb4 -> A8   D8
int sense4 = 9;       // pb5 -> A9   D9

int elEnable = 20;    // pf5 -> A2   D20

int detect = 5;

int rxled = 30;
int txled = 22;
int rstled = 7;

// This is hacky.  
// Work around from this changing when the program is loaded from the IDE.  
int capSenseFrequency = 1;

// State 

#define channelCount 4
int switched[] = {0, 0, 0, 0};
bool glow[] = {true, true, true, true};
bool allGlow[] = {true, true, true, true};
bool noGlow[] = {false, false, false, false};

#define prevValSize 50
int prevVal[prevValSize][channelCount];

#define sampleCount 3
int sampleAll[channelCount][sampleCount];
int sampleAllVal[channelCount];

int samples[sampleCount];
int sampleVal = 0;

// Helpers 

int tempGlow[channelCount];
byte tempGlow_unroll[channelCount * 2];
void switchOutputs(bool* glow) { 
  // Make pins on the layout correspond to the pins on the switch 
  tempGlow[2] = glow[0];
  tempGlow[3] = glow[1];
  tempGlow[1] = glow[2];
  tempGlow[0] = glow[3];

  // Process bit flips ahead of sending serial signal 
  PORTF = PORTF | B10000000;
  for (int i = 0; i < 8; i++) {
    if (tempGlow[i / 2] == 1) { 
      tempGlow_unroll[i] = PORTF | B01000000;
    }
    else { 
      tempGlow_unroll[i] = PORTF & B10111111;
    }
  }
  
  for (int i = 0; i < 8; i++) {
    PORTF = tempGlow_unroll[i];
    //   digitalWrite(hvClock, LOW); 
    PORTF = PORTF & B01111111;
    //   digitalWrite(hvClock, HIGH);
    PORTF = PORTF | B10000000;
  }

  
}


void switchHalfOutputsA() { 
  for (int i = 0; i < 4; i++) {
    PORTF = PORTF | B01000000;
    PORTF = PORTF & B01111111;
    PORTF = PORTF | B10000000;
    
    PORTF = PORTF & B10111111;
    PORTF = PORTF & B01111111;
    PORTF = PORTF | B10000000;
  }
}

void switchHalfOutputsB() { 
  for (int i = 0; i < 4; i++) {
    PORTF = PORTF & B10111111;
    PORTF = PORTF & B01111111;
    PORTF = PORTF | B10000000;
    
    PORTF = PORTF | B01000000;
    PORTF = PORTF & B01111111;
    PORTF = PORTF | B10000000;
  }
}

// This could be fancier...
void sort(int *a, int len)
{
   for (int i = 1; i < len; ++i)
   {
     int j = a[i];
     int k;
     for (k = i - 1; (k >= 0) && (j < a[k]); k--)
     {
       a[k + 1] = a[k];
     }
     a[k + 1] = j;
   }
}

// This mutates the input array! Beware!
float getMedian(int* array, int len) { 
//  int len = sizeof(array);
  sort(array, len); 
  float tempVal = 0;
  return array[(len / 2) + 1];
  
//  for (int i = (len / 2) - 1; i < (len / 2) + 2; i++) {
//    tempVal += array[i];
//  }
//  return tempVal / 3;
}

void ledsOff() {
  digitalWrite(rxled, LOW);
  digitalWrite(txled, LOW);
  digitalWrite(rstled, LOW);
}

void ledsOn() {
  digitalWrite(rxled, HIGH);
  digitalWrite(txled, HIGH);
  digitalWrite(rstled, HIGH);
}

unsigned long microsdelay;

// Arduino I/O is real slow.  Opaque bitwise port manipulation ftw!
int adcSensePorts[] = {B00000000, B00000001, B00000011, B00000100};

// this is internal, use the interface functions below!
bool toSense_internal[] = {true, true, true, true};
int senseResults_internal[] = {0, 0, 0, 0};

void initSense_withResistor(){ 
 // listen to resistor

    switchOutputs(noGlow);
    //  digitalWrite(elEnable, HIGH);
    PORTF = PORTF | B00100000;
    delayMicroseconds(100);
    
//    microsdelay = micros();
    while(bitRead(PINC, 6));
    switchOutputs(allGlow);
    switchHalfOutputsA();
    while(!bitRead(PINC, 6));
    while(bitRead(PINC, 6));
    while(!bitRead(PINC, 6));
    while(bitRead(PINC, 6));


    switchOutputs(noGlow);
    delayMicroseconds(30);

//    microsdelay = micros() - microsdelay;
}

int spinner = 0;
void initSense_withoutResistor() { 
    // Hacky.  Allows charge to disappate faster - kills flicker.

//    switchOutputs(allGlow);
//    microsdelay = 0;
//    while(bitRead(PINC, 6) || microsdelay < 20){
//      microsdelay++;
//    }
    // digitalWrite(elEnable, LOW);
    PORTF = PORTF & B11011111;
    for (int i = 0; i < 26; i++) {
      switchHalfOutputsA();
      spinner = 0;
      while(spinner < 50) { 
        spinner++;
      }
      
      switchHalfOutputsB();
      spinner = 0;
      while(spinner < 50) { 
        spinner++;
      }
    }

    // Make EL lamps float 
    switchOutputs(noGlow);
    delayMicroseconds(30);
}

void senseChannels_internal(int chargeDelay_micros = 1000) { 
    noInterrupts();
    
    // make sure pwm oscillator is running.
    pwm613configure(capSenseFrequency);
    pwmSet13(127);
    
    // digitalWrite(clearPin, HIGH);
    PORTD = PORTD | B10000000;

    if (chargeDelay_micros < 1) {
      chargeDelay_micros = 1000;
    }
    
// Set ADC initialization bits - make sure things haven't gotten misconfigured.  
    // 6: right adjust bits // last 5 bits select ADC.
    ADMUX = adcSensePorts[0];
    // high speed mode / 0 / analog selection, extra bit.
    ADCSRB = B10100000;
    // disable adc
    ADCSRA = B00000110;

    initSense_withoutResistor();

// Begin measurement sequence
    // digitalWrite(clearPin, LOW);
    PORTD = PORTD & B01111111;
    
    delayMicroseconds(chargeDelay_micros);
// start measurement
    for(int i = 0; i < 4; i++) {
      if (toSense_internal[i]) {
        // enable / start / auto trigger / interrupt flag / interrupt enable /// scale /// p.315
        ADCSRA = B11000110;
        delayMicroseconds(50);
        ADMUX = adcSensePorts[(i + 1) % 4];
        while ((ADCSRA & B01000000));
        sampleVal = ADCL;    // store lower byte ADC
        sampleVal += ADCH << 8;  // store higher bytes ADC
        senseResults_internal[i] = sampleVal;
      }
      else { 
        ADMUX = adcSensePorts[(i + 1) % 4];
        senseResults_internal[i] = -1;
      }
//      Serial.print(sampleVal);
//      Serial.print(", "); 
    }
//    Serial.println(" ");

   //  digitalWrite(elEnable, HIGH);
   PORTF = PORTF | B00100000;
   interrupts();
   //    digitalWrite(clearPin, HIGH);
   PORTD = PORTD | B10000000;
   return;
}

// Sensing interfaces! 
// Note that the channel here corresponds to the number written on the board, 
// which is shifted up by one from the indicies in the senseResults array.
int senseChannel(int channel, int chargeDelay_micros = 1000, bool isGlow = true) {
  if (channel < 0 || channel > 4) { 
    Serial.println("Warning: you might not be measuring the channel you intend to!");
    ledsOn();
  }
  channel = (channel - 1) % 4; // sanitize your inputs. 
  for (int i = 0; i < 4; i++) { 
    toSense_internal[i] = false; 
  }
  toSense_internal[channel] = true;
  senseChannels_internal(chargeDelay_micros);
  if (isGlow) { 
    switchOutputs(glow);
  }
  return senseResults_internal[channel];
}

int* senseAll(int chargeDelay_micros = 1000, bool isGlow = true) {
  for (int i = 0; i < 4; i++) { 
    toSense_internal[i] = true; 
  }
  senseChannels_internal(chargeDelay_micros);
  
  if (isGlow) { 
    switchOutputs(glow);
  }
  return senseResults_internal;
}

// The ardunio IDE insists on changing the frequency of the fast pwm when the board is programmed
// This is a hack that accounts for that.  
void ensureCorrectFrequency() {
  switchOutputs(noGlow);
  ledsOff();

  capSenseFrequency = PWM187k;
  int read187k = senseChannel(1, 5000);
  read187k = senseChannel(1, 5000);
  read187k = senseChannel(1, 5000);
//  Serial.print("187: ");
//  Serial.println(read187k);
  
  
  capSenseFrequency = PWM94k;
  int read94k = senseChannel(1, 5000);
  read94k = senseChannel(1, 5000);
  read94k = senseChannel(1, 5000);
//  Serial.print("94: ");
//  Serial.println(read94k);

  if (read94k > read187k) { 
    capSenseFrequency = PWM94k;
    digitalWrite(rxled, HIGH);
    if (read94k > 150) {
      digitalWrite(txled, HIGH);
    }
    delay(200);
  }
  else{ 
    capSenseFrequency = PWM187k;
    digitalWrite(rstled, HIGH);
    if (read187k > 150) {
      digitalWrite(txled, HIGH);
    }
    delay(200);
  }
  ledsOff();
}

//////////~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~///////////

void setup() {
  Serial.begin(9600);  

  // hvSwitch communication
  pinMode(hvDigitalIn, OUTPUT);
  pinMode(hvClock, OUTPUT);

  pinMode(clearPin, OUTPUT);
  digitalWrite(clearPin, LOW);

  pinMode(10, OUTPUT);

 
  pinMode(elEnable, OUTPUT);
  digitalWrite(elEnable, HIGH);

  pinMode(detect, INPUT);

  pinMode(sense1, INPUT);
  pinMode(sense2, INPUT);
  pinMode(sense3, INPUT);
  pinMode(sense4, INPUT);

  pinMode(rxled, OUTPUT);
  pinMode(txled, OUTPUT);
  pinMode(rstled, OUTPUT);   
  
  analogReference(DEFAULT);
  
  // this should be called after the clear pin is set low
  ensureCorrectFrequency();
}

int val = 0;

int minimum = 10000;
int maximum = 0;

#define senseSize 31
int senseHistory[channelCount][senseSize];
int senseHistoryTemp[senseSize];
int senseHistoryIter = 0;

bool isTouched = false;
bool whereTouched[] = {false, false, false, false};

float change = 0;

bool shouldGlow = true;

bool isOn = true;
bool justSwitched = false;
int switchedCount = 0;

int* senseResults;

#define smoothedSenseSize 25
int smoothedSenseHistory[channelCount][smoothedSenseSize];
int smoothedSenseIter = 0;


#define smoothedTouchWindow 4
float smoothedSampleMedian = 0;
float smoothedSampleMin = 0;
float smoothedSampleNew = 0;
int smoothedSampleTemp[smoothedSenseSize - smoothedTouchWindow];

int channelMean[] = {0, 0, 0, 0};
int chanThreshold[] = {1,1,1,1};

void loop() {
    senseResults = senseAll(1500, true);
    
    smoothedSenseIter = (smoothedSenseIter + 1) % smoothedSenseSize;


    // Get new smoothed value
    Serial.print("Signal: ");
    for (int channel = 0; channel < channelCount; channel++) { 
      senseHistory[channel][senseHistoryIter] = senseResults[channel];
      senseHistoryIter++;
      if (senseHistoryIter == senseSize) { 
        senseHistoryIter = 0;
      }

      for (int i = 0; i < senseSize; i++) {
        senseHistoryTemp[i] = senseHistory[channel][i];
      }

      int tempVal = 0;
      sort(senseHistoryTemp, senseSize);
  
      for (int i = 10; i < 13; i++) {
        tempVal += senseHistoryTemp[i];
      }

      smoothedSenseHistory[channel][smoothedSenseIter] = tempVal / 3;

      Serial.print(smoothedSenseHistory[channel][smoothedSenseIter]);
      Serial.print(", ");
    }

    for (int channel = 0; channel < channelCount; channel++) { 
      channelMean[channel] = (channelMean[channel] + 
                              smoothedSenseHistory[channel][(smoothedSenseIter + 1 + smoothedSenseSize) % smoothedSenseSize]) / 2;
      Serial.print(", ");
      Serial.print(smoothedSenseHistory[channel][smoothedSenseIter] - channelMean[channel]);
      Serial.print(", ");

      if (channelMean[channel] - smoothedSenseHistory[channel][smoothedSenseIter] > 25) { 
        chanThreshold[channel] = 3;
      }

      if (switchedCount == 0 && 
          channelMean[channel] - smoothedSenseHistory[channel][smoothedSenseIter] > chanThreshold[channel] && 
          channelMean[channel] - smoothedSenseHistory[channel][(smoothedSenseIter + smoothedSenseSize - 1) % smoothedSenseSize] > chanThreshold[channel]){
        isTouched = true;
        whereTouched[channel] = true;
      }
      
    }
      
    Serial.println();
    

    if (isTouched && switchedCount == 0) { 
      isTouched = false;
      for (int chan = 0; chan < channelCount; chan++) {
        if (whereTouched[chan]) { 
          Serial.println(chan);
       //   delay(1000);
          whereTouched[chan] = false;
          glow[chan] = !glow[chan];
        }
      }
      switchedCount = 2 * smoothedSenseSize;
      if (switchedCount < 50) {
        switchedCount = 50;
      }
      smoothedSenseIter = 0;
    }
    
    switchOutputs(glow);

    if (switchedCount > 0) {
      switchedCount--;
    }
    
    delay(11);
}
