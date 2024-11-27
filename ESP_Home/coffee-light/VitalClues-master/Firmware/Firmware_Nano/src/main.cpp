#include <Arduino.h>


// include libraries we need
#include <OneWire.h>
// #include <DallasTemperature.h>

// #define ONE_WIRE_BUS 2 // temp. sensor connected on D2
#define CONTROL_PIN 3 // relay controlled via D3

// which analog pin to connect
#define THERMISTORPIN A0
// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 5
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3960
// the value of the 'other' resistor
#define SERIESRESISTOR 9970

float currentTemp;
float targetTemperature = 38; // in Celcius, do not use more than 50!!!
float waterTemp; // temperature measured in anus

// PID variables
float err; // difference between measured and set point
float previous_err = 0;
float integral = 0;
float derivative = 0;
const unsigned char nInt = 20;
float errContainer[nInt];
unsigned char iErr; // runs from 1 to 10
const float pValue = 90.0; // proportional part of controller
const float dValue = 0; // differential part of controller
const float iValue = 10; // integral part of controller

float pwmValue = 0; // set initial heating to 0
int iPwmValue = 0; // inted pwm value

// create temperature sensor class member from library
// OneWire oneWire(ONE_WIRE_BUS);
// DallasTemperature TempSensor(&oneWire);


// function to read out smaller sensor placed in anus of animal
float getAnalogTemp(){
  float average = 0;
  // take N samples in a row and sum them up
  for (int i = 0; i< NUMSAMPLES; i++)
    average += analogRead(THERMISTORPIN);
  average /= NUMSAMPLES;

  // convert the value to resistance
  average = 1023 / average - 1;
  average = SERIESRESISTOR / average;

  waterTemp = average / THERMISTORNOMINAL;     // (R/Ro)
  waterTemp = log(waterTemp);                  // ln(R/Ro)
  waterTemp /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  waterTemp += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  waterTemp = 1.0 / waterTemp;                 // Invert
  waterTemp -= 273.15;                         // convert to C
  return waterTemp;
}

void setup(void) {
  Serial.begin(9600);
  analogReference(EXTERNAL);
  pinMode(CONTROL_PIN, OUTPUT); // set relay pin as digital output
  analogWrite(CONTROL_PIN, pwmValue);

  // reset error container to 0s
  for (iErr = 0; iErr < nInt; iErr++)
    errContainer[iErr] = 0;

  iErr = 0;
}


void loop(void) {

  waterTemp = getAnalogTemp();

  err = targetTemperature - waterTemp; // calculate error
  errContainer[iErr] = err;
  if (iErr >= nInt)
    iErr = 0;
  // calculate integral over last 10 measurements
  integral = 0;
  for(unsigned char i = 0; i < nInt; i++)
    integral += errContainer[i];
  // integral = integral + err; // update integral value

  derivative = err - previous_err; // update derivative error
  pwmValue = pValue * err + dValue * derivative + iValue * integral;
  previous_err = err;

  iPwmValue = pwmValue;
  if (pwmValue < 0){
    pwmValue = 0;
  }

  if (pwmValue > 255){
    pwmValue = 255;
  }

  analogWrite(CONTROL_PIN, pwmValue);

  Serial.print(waterTemp);
  Serial.print(" ");
  Serial.print(pwmValue);
  Serial.print(" ");
  Serial.println(targetTemperature);
  // Serial.println(targetTemperature);


  delay(500); // no need to check more than 2x per second...
}
