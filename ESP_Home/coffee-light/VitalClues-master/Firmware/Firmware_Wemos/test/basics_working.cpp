#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP8266WiFi.h>

// replace with your channel’s thingspeak API key and your SSID and password
String apiKey = "AJ60CMZMC275VGXY";
const char* ssid = "RazanskyLab";
const char* password = "WeLoveOptoacoustics";
const char* server = "api.thingspeak.com";

#define ONE_WIRE_BUS D2 // temp. sensor connected on D2
#define CONTROL_PIN D3 // relay controlled via D3

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

float maxPadTemp = 50;
float currentTemp;
bool heatPadOn = false;
float targetTemperature = 37; // in Celcius, do not use more than 50!!!
float analTemp; // temperature measured in anus
float padTemp;
float faultTemperature = -120;

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

unsigned long previousDataSend = 0;     // will store last time LED was updated
const long updateIOTInterval = 20000;  // interval at which to blink (milliseconds)

// create temperature sensor class member from library
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature TempSensor(&oneWire);
WiFiClient client;


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

  analTemp = average / THERMISTORNOMINAL;     // (R/Ro)
  analTemp = log(analTemp);                  // ln(R/Ro)
  analTemp /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  analTemp += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  analTemp = 1.0 / analTemp;                 // Invert
  analTemp -= 273.15;                         // convert to C
  return analTemp;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// function to read out digital sensor placed on pad
float getDigitalTemp(){
  // update temperature readings.
  TempSensor.requestTemperatures();
  currentTemp = TempSensor.getTempCByIndex(0);
  return currentTemp;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void setup(void) {
  Serial.begin(9600);

  // connect to WIFI
  WiFi.begin(ssid, password);

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
  delay(500);
  Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  TempSensor.begin();   // Start TempSensor
  TempSensor.setResolution(12); // reduce resolution to reduce noise

  pinMode(CONTROL_PIN, OUTPUT); // set relay pin as digital output
  analogWrite(CONTROL_PIN, pwmValue);

  // reset error container to 0s
  for (iErr = 0; iErr < nInt; iErr++)
    errContainer[iErr] = 0;

  iErr = 0;
}

void loop(void) {

  analTemp = getAnalogTemp();
  padTemp = getDigitalTemp();

  err = targetTemperature - analTemp; // calculate error
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

  // if pad termperature is too high, disable, otherwise we carbonize the mouse
  if (padTemp > maxPadTemp)
    pwmValue = 0;

  analogWrite(CONTROL_PIN, pwmValue);

  // serial print values
  Serial.print(analTemp);
  Serial.print(" ");
  Serial.print(padTemp);
  Serial.print(" ");
  Serial.println(targetTemperature);

  if (millis() - previousDataSend >= updateIOTInterval) {
    Serial.print("Sending data to the cloud...");
    if (client.connect(server,80))
    {
      String postStr = apiKey;
      postStr +="&field1=";
      postStr += String(analTemp);
      postStr +="&field2=";
      postStr += String(padTemp);
      postStr +="&field3=";
      postStr += String(targetTemperature);
      postStr +="&field4=";
      postStr += String(pwmValue);
      postStr += "\r\n\r\n";

      client.print("POST /update HTTP/1.1\n");
      client.print("Host: api.thingspeak.com\n");
      client.print("Connection: close\n");
      client.print("X-THINGSPEAKAPIKEY: "+apiKey+"\n");
      client.print("Content-Type: application/x-www-form-urlencoded\n");
      client.print("Content-Length: ");
      client.print(postStr.length());
      client.print("\n\n");
      client.print(postStr);
      client.stop();
    }
    else {
      Serial.println("failed!");
    }
    previousDataSend = millis();
    Serial.println("done!");
  }
}
