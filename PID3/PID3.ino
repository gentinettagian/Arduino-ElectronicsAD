
/*
 * Uses a PID controller to control the temperature
 * of our circuit. 
 * 
 * We measure the temperature of the
 * heated circuit with the thermistor. 
 * 
 * The PID controller aims to cool the circuit down to
 * 25 degrees Celcius using the connected fan.
 * 
 * inspired by
 * http://electronoobs.com/eng_arduino_tut100_code1.php
 * 
 */

#include <Wire.h>
#include "rgb_lcd.h"

rgb_lcd lcd;

#define INTERVAL_MICROS         1000000 //update time
#define OVERFLOW             4000000000 //at this time we reset to 0


#define TOLERANCE                   0.1 //tolerance for integral part

// PID parameters (to tune)
double kp = 100; //proportional
double ki = 30; //integral
double kd = 50; //derivative

double PID_p, PID_i, PID_d,PID_total;

//Temperature values
double T,Tdiff;
double Tref = 25; //Goal temperature

unsigned long last_time = 0; //last timestamp 
unsigned int count = 0; //cycle counter

const int fanPin = 3; // Pin to controll fan speed
double power = 100;

const int opAmpPin = A2; // Pin for op-Amp input
int opAmpValue = 0; // measured voltage
const int grovePin = A0; // Pin for Grove Sensor input
int groveValue = 0; // measured voltage

const int R0 = 100000; // zero resistance R_0 in Ohm
const int B = 6700; // nominal B-constnat in Kelvin
const int Bref = 4275; // Grove B-constant in Kelvin
const float T0 = 25 + 273.15; // room temperature in Kelvin


int colorR = 255;
int colorG = 255;
int colorB = 255;

void setup() {
  //set up the LCD's number of columns and rows:
  lcd.begin(16,2);
  lcd.clear();
  lcd.print("Temp:   Speed:");
  Serial.begin(9600);
  lcd.setRGB(colorR,colorG,colorB);
  analogWrite(fanPin,power);


}

void loop() {
  
  unsigned long now = micros();
  if (now > (last_time + INTERVAL_MICROS)) {

    // Calculating the circuit temperature
    opAmpValue = analogRead(opAmpPin);
    float R = (1023.0/opAmpValue - 1)*R0; // resistance in Ohm
    T = 1.0/(log(R/R0)/B + 1/T0) - 273.15; // temperature in Celsius

    double TdiffNew = T - Tref;

    PID_p = kp * TdiffNew; //proportional part
    PID_d = kd * (TdiffNew - Tdiff)/INTERVAL_MICROS * 1000000; // differential part

    //The if statement is to prevent over/undershoot
    if (-TOLERANCE > TdiffNew && TdiffNew > TOLERANCE) {
      PID_i = PID_i + (ki * TdiffNew);
    } else {
      PID_i = 0;
    }
    
    
    Tdiff = TdiffNew;

    //compute the PID
    PID_total = PID_p + PID_d + PID_i;
    power += PID_total;

    //keep power in range
    if (power < 0) {
      power = 0;
    } else if (power > 255) {
      power = 255;
    }
    
    //controll the fan
    analogWrite(fanPin,power);

    
    lcd.setCursor(0,1);
    lcd.print(T);
    lcd.setCursor(8,1);
    lcd.print(power);
    lcd.setRGB(colorR,colorG,colorB);

    /* Uncomment to tune
    Serial.print(PID_p);
    Serial.print(" ");
    Serial.print(PID_i);
    Serial.print(" ");
    Serial.print(PID_d);
    Serial.print(" ");
    */

    //Output to python
    Serial.print(T);
    Serial.print(" ");
    Serial.print(Tref);
    Serial.print(" ");
    Serial.println(power);
 
    if (now > OVERFLOW) { //make sure not to overflow
      now = 0;
    }

    last_time = now;
    count++;
  }
}
