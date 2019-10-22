
/*
 * Turns on the fan at a constant speed (power)
 * and prints the current RPM as well as the 
 * temperature on the LCD display every second.
 */

#include <Wire.h>
#include "rgb_lcd.h"

rgb_lcd lcd;

#define INTERVAL_MICROS         2000000 //update time
#define OVERFLOW             4000000000 //at this time we reset to 0
#define FAN_INTERVAL_MICROS        10   //measuring fan pulse frequency

#define FAN_THRESHOLD               500 //above this threshold the fan pulse is high

const int power = 255; //We measure equilibrium at this point (this is changed)

unsigned long last_time = 0; //last timestamp 
unsigned int last_fan_pulse = 0; //last fan pulse test time

const int fanPin = 3; // Pin to control fan

const int analogInPin = A2; // Pin for sensor input
int sensorValue = 0; // measured voltage

const int fanPulsePin = A3; // Pin for fan pulse
unsigned int pulse = 0; //The current fan pulse
bool pulse_high = true; //true if the fan pulse is above threshold
unsigned int pulse_counter = 0; //counts pulse per minute

int threshold = 30; // temperature threshold
const int threshold_diff = 5; //difference between threshold and lower threshold 

int RPM = 0; //the current rotations per minute

const int R0 = 100000; // zero resistance R_0 in Ohm
const int B = 6700; // nominal B-constnat in Kelvin
const float T0 = 25 + 273.15; // room temperature in Kelvin

int colorR = 255;
int colorG = 255;
int colorB = 255;

void setup() {
  //set up the LCD's number of columns and rows:
  lcd.begin(16,2);
  Serial.begin(9600);
  lcd.setRGB(colorR,colorG,colorB);
  lcd.setCursor(0,0);
  lcd.print("RPM:    T:");
  analogWrite(fanPin,power);
}

void loop() {

  if (power < 256) {
    unsigned long now = micros();
    if (now > (last_time + INTERVAL_MICROS)) {

      sensorValue = analogRead(analogInPin);
      float R = (1023.0/sensorValue - 1)*R0; // resistance in Ohm
      float T = 1.0/(log(R/R0)/B + 1/T0) - 273.15; // temperature in Kelvin
      
      //Printing the current RPM
      int rotations = pulse_counter / 4;
      Serial.print(T);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("RPM:    T:");
      lcd.setCursor(0,1);
      lcd.print(rotations * 30); //measuring over 2 seconds
      lcd.setCursor(8,1);
      lcd.print(T);
      pulse_counter = 0;

      if (now > OVERFLOW) { //make sure not to overflow
        now = 0;
      }

      last_time = now;

      //Fan pulse
      
    
    }
    if (now > FAN_INTERVAL_MICROS + last_fan_pulse) {
        pulse = analogRead(fanPulsePin);
        bool test_pulse_high = pulse > FAN_THRESHOLD;
        if (test_pulse_high != pulse_high) { //Test if pulse has changed
          pulse_high = test_pulse_high;
          pulse_counter++;
        }
    }
    
  } else {
    analogWrite(fanPin,0);
  }
    

  
  
}
