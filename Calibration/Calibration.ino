
/*
 * Compares the measured temperatures of the self built Op-Amp temperature sensor 
 * with the reference Grove Sensor. Goal is to calibrate the B-value, so that
 * the temperatures coincide. 
 * The B-value can be changed using the Potentiometer. Once the values are in range,
 * the display turns green.
 */

#include <Wire.h>
#include "rgb_lcd.h"

rgb_lcd lcd;

#define INTERVAL_MICROS         1000000 //update time
#define OVERFLOW             4000000000 //at this time we reset to 0

//Set the range for the B-value
#define Bmin                      5000 
#define Bmax                      7000

unsigned long last_time = 0; //last timestamp 
unsigned int count = 0; //cycle counter

const int opAmpPin = A2; // Pin for op-Amp input
int opAmpValue = 0; // measured voltage
const int grovePin = A0; // Pin for Grove Sensor input
int groveValue = 0; // measured voltage

const int potentInPin = A1; // Pin for potentiometer
int potentiometer = 0; //potentiometer input

const int R0 = 100000; // zero resistance R_0 in Ohm
int B = 4600; // nominal B-constnat in Kelvin (for Op-Amp)
const int Bref = 4275; // Grove B-constant in Kelvin
const float T0 = 25 + 273.15; // room temperature in Kelvin


int colorR = 255;
int colorG = 255;
int colorB = 255;

void setup() {
  //set up the LCD's number of columns and rows:
  lcd.begin(16,2);
  lcd.print("Topamp: Tgrove:");
  Serial.begin(9600);
  lcd.setRGB(colorR,colorG,colorB);
}

void loop() {
 
  //adjusting the B-value
  potentiometer = analogRead(potentInPin);
  B = map(potentiometer,0,1023,Bmin,Bmax);
  
  unsigned long now = micros();
  if (now > (last_time + INTERVAL_MICROS)) {
    opAmpValue = analogRead(opAmpPin);
    float R = (1023.0/opAmpValue - 1)*R0; // resistance in Ohm
    float T = 1.0/(log(R/R0)/B + 1/T0) - 273.15; // temperature in Kelvin
    Serial.print("Temperature Op-Amp: ");
    Serial.print(T);
    
    groveValue = analogRead(grovePin);
    float Rref = (1023.0/groveValue - 1)*R0; // resistance in Ohm
    float Tref = 1.0/(log(Rref/R0)/Bref + 1/T0) - 273.15; // temperature in Kelvin
    Serial.print("Temperature Grove: ");
    Serial.print(T);

    lcd.setCursor(0,1);
    lcd.print(T);
    lcd.setCursor(8,1);
    lcd.print(Tref);
    
    //Turning the display backlight red as a warning if T > threshold
    Serial.print(", B value: ");
    Serial.println(B);
    if (abs(T-Tref) < 0.005) {
      colorR = 0;
      colorB = 0;
    } else {
      colorR = 255;
      colorB = 255;
    }
    lcd.setRGB(colorR,colorG,colorB);
 
    if (now > OVERFLOW) { //make sure not to overflow
      now = 0;
    }

    last_time = now;
    count++;
  }
}
