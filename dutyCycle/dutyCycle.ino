
/*
 * Slowly turns on the fan and measures the rotations per minute
 * for the different input voltages. The result is printed on the
 * display and into Serial monitor.
 */

#include <Wire.h>
#include "rgb_lcd.h"

rgb_lcd lcd;

#define INTERVAL_MICROS         2000000 //update time
#define OVERFLOW             4000000000 //at this time we reset to 0
#define FAN_INTERVAL_MICROS        10   //measuring fan pulse frequency

#define FAN_THRESHOLD               500 //above this threshold the fan pulse is high

unsigned long last_time = 0; //last timestamp 
unsigned int last_fan_pulse = 0; //last fan pulse test time

const int fanPin = 3; // Pin to control fan

const int fanPulsePin = A3; // Pin for fan pulse
unsigned int pulse = 0; //The current fan pulse
bool pulse_high = true; //true if the fan pulse is above threshold
unsigned int pulse_counter = 0; //counts pulse per minute

int threshold = 30; // temperature threshold
const int threshold_diff = 5; //difference between threshold and lower threshold 

int RPM = 0; //the current rotations per minute
int power = -10; //the input power, ranges from 0 to 255 (-10 to have time to prepare)

int colorR = 255;
int colorG = 255;
int colorB = 255;

void setup() {
  //set up the LCD's number of columns and rows:
  lcd.begin(16,2);
  Serial.begin(9600);
  lcd.setRGB(colorR,colorG,colorB);
  lcd.print("RPM:");
  analogWrite(fanPin,0);
}

void loop() {

  if (power < 256) {
    unsigned long now = micros();
    if (now > (last_time + INTERVAL_MICROS)) {
      
      //Printing the current RPM
      int rotations = pulse_counter / 4;
      Serial.print(power);
      Serial.print(" ");
      Serial.println(rotations * 30); //measuring over 2 seconds
      lcd.setCursor(1,0);
      lcd.print(rpm)
      pulse_counter = 0;

      power++;
      if (power > 0) {
        analogWrite(fanPin,power);
      }
 
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
