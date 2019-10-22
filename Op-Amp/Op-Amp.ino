
/*
 * Reads the voltage from the thermistor (temperature sensor). 
 * It than calculates the corresponding resistance and finally converts
 * the resistance into the temperature in °C.
 * The measured temperature is shown on a connected Grove RGB LCD display.
 * 
 * Further, a potentiometer is connected to the system which controls a 
 * maximal temperature threshold. Once this threshold is reached, the 
 * display turns red.
 */

#include <Wire.h>
#include "rgb_lcd.h"

rgb_lcd lcd;

#define INTERVAL_MICROS         1000000 //update time
#define OVERFLOW             4000000000 //at this time we reset to 0

unsigned long last_time = 0; //last timestamp 
unsigned int count = 0; //cycle counter

const int analogInPin = A2; // Pin for sensor input
int sensorValue = 0; // measured voltage

const int potentInPin = A1; // Pin for potentiometer
int potentiometer = 0; //potentiometer input

const int R0 = 100000; // zero resistance R_0 in Ohm
const int B = 6700; // nominal B-constnat in Kelvin
const float T0 = 25 + 273.15; // room temperature in Kelvin

int threshold = 30; // temperature threshold

int colorR = 255;
int colorG = 255;
int colorB = 255;

void setup() {
  //set up the LCD's number of columns and rows:
  lcd.begin(16,2);
  lcd.print("Temp:   Tmax:");
  Serial.begin(9600);
  lcd.setRGB(colorR,colorG,colorB);
}

void loop() {
 
  //adjusting the threshold
  potentiometer = analogRead(potentInPin);
  threshold = map(potentiometer,0,1023,20,40);
  lcd.setCursor(8,1);
  lcd.print(threshold);
  
  unsigned long now = micros();
  if (now > (last_time + INTERVAL_MICROS)) {
    sensorValue = analogRead(analogInPin);
    float R = (1023.0/sensorValue - 1)*R0; // resistance in Ohm
    float T = 1.0/(log(R/R0)/B + 1/T0) - 273.15; // temperature in Kelvin

    /* Serial printer
    Serial.print("Time: ");
    Serial.print(count);
    Serial.print("s, Temperature: ");
    Serial.print(T);
    */

    // Printer for easy Python analysis:
    Serial.print(T);
    Serial.print(" ");

    lcd.setCursor(0,1);
    lcd.print(T);
    
    //Turning the display backlight red as a warning if T > threshold
    //Serial.print(" above threshold (50 if yes): ");
    if (T>threshold) {
      colorG = 0;
      colorB = 0;
      Serial.println(50);
    } else {
      colorG = 255;
      colorB = 255;
      Serial.println(0);
    }
    lcd.setRGB(colorR,colorG,colorB);
 
    if (now > OVERFLOW) { //make sure not to overflow
      now = 0;
    }

    last_time = now;
    count++;
  }
}
