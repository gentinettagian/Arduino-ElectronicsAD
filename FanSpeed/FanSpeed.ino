
/*
 * Reads the voltage from a Grove Temperature Sensor. 
 * It than calculates the corresponding resistance and finally converts
 * the resistance into the temperature in °C.
 * The measured temperature is shown on a connected Grove RGB LCD display.
 * 
 * Further, a potentiometer is connected to the system which controls a 
 * maximal temperature threshold. Once this threshold is reached, the 
 * display turns red.
 * 
 * Reads out the fan speed by counting the pulses and converting to RPM.
 */

#include <Wire.h>
#include "rgb_lcd.h"

rgb_lcd lcd;

#define INTERVAL_MICROS         1000000 //update time
#define OVERFLOW             4000000000 //at this time we reset to 0
#define FAN_INTERVAL_MICROS        1000 //measuring fan pulse frequency

#define FAN_THRESHOLD               500 //above this threshold the fan pulse is high

unsigned long last_time = 0; //last timestamp 
unsigned int count = 0; //cycle counter
unsigned int last_fan_pulse = 0; //last fan pulse test time

const int fanPin = 3; // Pin to control fan

const int analogInPin = A2; // Pin for sensor input
int sensorValue = 0; // measured voltage

const int potentInPin = A1; // Pin for potentiometer
unsigned int potentiometer = 0; //potentiometer input

const int fanPulsePin = A3; // Pin for fan pulse
unsigned int pulse = 0; //The current fan pulse
bool pulse_high = true; //true if the fan pulse is above threshold
unsigned int pulse_counter = 0; //counts pulse per minute

const int R0 = 100000; // zero resistance R_0 in Ohm
const int B = 6700; // nominal B-constnat in Kelvin
const float T0 = 25 + 273.15; // room temperature in Kelvin

int threshold = 30; // temperature threshold
const int threshold_diff = 5; //difference between threshold and lower threshold 



int colorR = 255;
int colorG = 255;
int colorB = 255;

void setup() {
  //set up the LCD's number of columns and rows:
  lcd.begin(16,2);
  lcd.print("Temp:   Tmax:");
  Serial.begin(9600);
  lcd.setRGB(colorR,colorG,colorB);
  analogWrite(fanPin,0);
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
    //The fan will turn on once the higher threshold is reached and turn off once the
    //temperature is back below the lower threshold.
    //Serial.print(" above threshold (50 if yes): ");
    if (T>threshold) {
      colorG = 0;
      colorB = 0;
      Serial.print(50);
      analogWrite(fanPin,255);
    } else if (T < threshold - threshold_diff) {
      colorG = 255;
      colorB = 255;
      Serial.print(0);
      analogWrite(fanPin,0);
    } else { //The display turns yellow once the fan is activated
      colorG = 255;
      Serial.print(25);
    }
    lcd.setRGB(colorR,colorG,colorB);


    //Printing the current RPM
    int rotations = pulse_counter / 4;
    Serial.print(" ");
    Serial.println(rotations * 60);
    pulse_counter = 0;
 
    if (now > OVERFLOW) { //make sure not to overflow
      now = 0;
    }

    last_time = now;
    count++;
    
  }

  //Fan pulse
  if (now > FAN_INTERVAL_MICROS + last_fan_pulse) {
    pulse = analogRead(fanPulsePin);
    bool test_pulse_high = pulse > FAN_THRESHOLD;
    if (test_pulse_high != pulse_high) { //Test if pulse has changed
      pulse_high = test_pulse_high;
      pulse_counter++;
    }
    
  }
  
}
