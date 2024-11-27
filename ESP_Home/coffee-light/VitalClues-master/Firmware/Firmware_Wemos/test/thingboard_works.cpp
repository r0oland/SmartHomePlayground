#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
// #include <WiFiEsp.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ThingsBoard.h>


#define WIFI_SSID   "RazanskyLab"
#define WIFI_PASS   "WeLoveOptoacoustics"
#define TOKEN "7DHHQiddU5wPhG5ZGEwP"
char thingsboardServer[] = "116.203.61.127";
WiFiClient client;
ThingsBoard tb(client);

int status = WL_IDLE_STATUS;
unsigned long lastSend;

// we get 30 writes per minute
const long updateIOTInterval = 0;  // interval at which to blink (milliseconds)
unsigned long previousDataSend = updateIOTInterval;     // will store last time LED was updated

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Pin definitions
#define ONE_WIRE_BUS D3 // temp. sensor connected on D2
#define CONTROL_PIN D4 // transistor controll pin
#define THERMISTORPIN A0

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// variables for analog & digital temperature measurements
const uint16_t THERMISTORNOMINAL = 10000;
const float TEMPERATURENOMINAL = 25.0;
const uint8_t N_SAMPLES = 10;
const uint16_t BCOEFFICIENT = 3960;
const uint16_t SERIESRESISTOR = 9970;
const float targetTemperature = 37.0;
const float faultTemperature = -120; // TODO make sure to use this!
float maxPadTemp = 50.0;
float currentTemp = 0.0;
float analTemp = 0.0; // temperature measured in anus
float padTemp = 0.0;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

// %%%%%%%%%%%%%%%
uint8_t lcd_clock[8] = {0x0, 0xe, 0x15, 0x17, 0x11, 0xe, 0x0};
uint8_t heart[8] = {0x0, 0xa, 0x1f, 0x1f, 0xe, 0x4, 0x0};
uint8_t check[8] = {0x0, 0x1 ,0x3, 0x16, 0x1c, 0x8, 0x0};


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// create all class members from libaries
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature TempSensor(&oneWire);
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// function to read out smaller sensor placed in anus of animal
float get_analog_temp(){
  float average = 0;
  // take N samples in a row and sum them up
  for (int i = 0; i< N_SAMPLES; i++)
    average += analogRead(THERMISTORPIN);
  average /= N_SAMPLES;

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
float get_digital_temp(){
  // update temperature readings.
  TempSensor.requestTemperatures();
  currentTemp = TempSensor.getTempCByIndex(0);
  return currentTemp;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// function to read out digital sensor placed on pad
float control_heat_pad(float currentTemp){
    err = targetTemperature - currentTemp; // calculate error
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
  return pwmValue;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void setup_wifi(){
  if(WiFi.status() != WL_CONNECTED){
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(WIFI_SSID);
    while(WiFi.status() != WL_CONNECTED){
      WiFi.begin(WIFI_SSID, WIFI_PASS);  // Connect to WPA/WPA2 network. Change this line if using open or WEP network
      Serial.print(".");
      delay(5000);
    }
    Serial.println("\nConnected.");
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!tb.connected()) {
    Serial.print("Connecting to ThingsBoard node ...");
    // Attempt to connect (clientId, username, password)
    if ( tb.connect(thingsboardServer, TOKEN) ) {
      Serial.println( "[DONE]" );
    } else {
      Serial.print( "[FAILED]" );
      Serial.println( " : retrying in 5 seconds" );
      // Wait 5 seconds before retrying
      delay( 5000 );
    }
  }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void send_iot_data(){
  static unsigned long previousDataSend = 0;
  if ((previousDataSend == 0) || (millis() - previousDataSend >= updateIOTInterval)) {
    tb.sendTelemetryFloat("analTemp", analTemp);
    tb.sendTelemetryFloat("padTemp", padTemp);
    tb.sendTelemetryFloat("pwmValue", pwmValue);
    previousDataSend = millis();
  }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void setup(void) {
  lcd.init();                      // initialize the lcd
  lcd.backlight();
  lcd.createChar(0, lcd_clock);
  lcd.createChar(1, heart);
  lcd.createChar(2, check);
  lcd.clear();
  lcd.home();
  delay(1000);

  // lcd.printByte(0); // clock
  // lcd.printByte(1); // heart
  // lcd.printByte(2); // check

  Serial.begin(9600);
  // wait for serial monitor to open
  while (!Serial);

  setup_wifi();

  TempSensor.begin();   // Start TempSensor
  TempSensor.setResolution(12); // reduce resolution to reduce noise

  pinMode(CONTROL_PIN, OUTPUT); // set relay pin as digital output
  analogWrite(CONTROL_PIN, pwmValue);

  // reset error container to 0s
  for (iErr = 0; iErr < nInt; iErr++){
    errContainer[iErr] = 0;
  }
  iErr = 0;

}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void loop(void) {
  status = WiFi.status();
  if ( status != WL_CONNECTED) {
    while ( status != WL_CONNECTED) {
      Serial.print("Attempting to connect to WPA SSID: ");
      Serial.println(WIFI_SSID);
      // Connect to WPA/WPA2 network
      status = WiFi.begin(WIFI_SSID, WIFI_PASS);
      delay(500);
    }
    Serial.println("Connected to AP");
  }

  if ( !tb.connected() ) {
    reconnect();
  }

  lcd.home();

  analTemp = get_analog_temp();
  padTemp = get_digital_temp();
  pwmValue = control_heat_pad(currentTemp);

  send_iot_data();

  // serial print values
  Serial.print(analTemp);
  Serial.print(" ");
  Serial.print(padTemp);
  Serial.print(" ");
  Serial.print(pwmValue);
  Serial.print(" ");
  Serial.println(targetTemperature);


  // FIXME use sprintf instead!
  // https://arduinobasics.blogspot.com/2019/05/sprintf-function.html
  lcd.setCursor(6, 0);
  lcd.print(" M:");
  lcd.print(analTemp,1);
  lcd.setCursor(0, 1);
  lcd.print("P:");
  lcd.print(padTemp,1);
  lcd.print(" PW:");
  lcd.print(pwmValue/255*100,0);
  lcd.print("%");

  tb.loop();
}
