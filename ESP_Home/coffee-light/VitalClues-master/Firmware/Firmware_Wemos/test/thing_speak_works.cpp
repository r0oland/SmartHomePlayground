#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP8266WiFi.h>
#include <ThingSpeak.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include "AdafruitIO_WiFi.h"

#define IO_USERNAME  "r0oland"
#define IO_KEY       "6dba701a1470d0c8e9c61c75100c8e3acf3c8955"
#define WIFI_SSID   "RazanskyLab"
#define WIFI_PASS   "WeLoveOptoacoustics"
AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);
AdafruitIO_Feed *padFeed = io.feed("padFeed");
AdafruitIO_Feed *mouseFeed = io.feed("mouseFeed");
AdafruitIO_Feed *pwmFeed = io.feed("pwmFeed");


#if defined(ARDUINO) && ARDUINO >= 100
#define printByte(args)  write(args);
#else
#define printByte(args)  print(args,BYTE);
#endif

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// ThingSpeak Connection & Wifi
#define SECRET_SSID "RazanskyLab"		// replace MySSID with your WiFi network name
#define SECRET_PASS "WeLoveOptoacoustics"	// replace MyPassword with your WiFi password
#define SECRET_CH_ID 859305			// replace 0000000 with your channel number
#define SECRET_WRITE_APIKEY "AJ60CMZMC275VGXY"

char ssid[] = SECRET_SSID;   // your network SSID (name)
char pass[] = SECRET_PASS;   // your network password
unsigned long myChannelNumber = SECRET_CH_ID;
const char * myWriteAPIKey = SECRET_WRITE_APIKEY;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Pin definitions
#define ONE_WIRE_BUS D3 // temp. sensor connected on D2
#define CONTROL_PIN D4 // transistor controll pin
#define THERMISTORPIN A0

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// variables for analog & digital temperature measurements
const uint16_t THERMISTORNOMINAL = 10000;
const float TEMPERATURENOMINAL = 25.0;
const uint8_t N_SAMPLES = 5;
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
uint8_t bell[8]  = {0x4, 0xe, 0xe, 0xe, 0x1f, 0x0, 0x4};
uint8_t note[8]  = {0x2, 0x3, 0x2, 0xe, 0x1e, 0xc, 0x0};
uint8_t lcd_clock[8] = {0x0, 0xe, 0x15, 0x17, 0x11, 0xe, 0x0};
uint8_t heart[8] = {0x0, 0xa, 0x1f, 0x1f, 0xe, 0x4, 0x0};
uint8_t duck[8]  = {0x0, 0xc, 0x1d, 0xf, 0xf, 0x6, 0x0};
uint8_t check[8] = {0x0, 0x1 ,0x3, 0x16, 0x1c, 0x8, 0x0};
uint8_t cross[8] = {0x0, 0x1b, 0xe, 0x4, 0xe, 0x1b, 0x0};
uint8_t retarrow[8] = {	0x1, 0x1, 0x5, 0x9, 0x1f, 0x8, 0x4};


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// misc variables
const long updateIOTInterval = 20000;  // interval at which to blink (milliseconds)
unsigned long previousDataSend = updateIOTInterval;     // will store last time LED was updated

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// create all class members from libaries
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature TempSensor(&oneWire);
WiFiClient  client;
// LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
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
    Serial.println(SECRET_SSID);
    while(WiFi.status() != WL_CONNECTED){
      WiFi.begin(ssid, pass);  // Connect to WPA/WPA2 network. Change this line if using open or WEP network
      Serial.print(".");
      delay(5000);
    }
    Serial.println("\nConnected.");
  }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void send_iot_data(){
  static unsigned long previousDataSend = 0;
  setup_wifi();
  if ((previousDataSend == 0) || (millis() - previousDataSend >= updateIOTInterval)) {
    // set the fields with the values
    ThingSpeak.setField(1, analTemp);
    ThingSpeak.setField(2, padTemp);
    ThingSpeak.setField(3, targetTemperature);
    ThingSpeak.setField(4, pwmValue);
    String myStatus = "TestStatus";
    ThingSpeak.setStatus(myStatus);

    // write to the ThingSpeak channel
    int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    if (x == 200){
      Serial.println("Channel update successful.");
    }
    else{
      Serial.println("Problem updating channel. HTTP error code " + String(x));
    }

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
  lcd.home();
  delay(1000);

  // lcd.printByte(0); // clock
  // lcd.printByte(1); // heart
  // lcd.printByte(2); // check

  lcd.print("Starting serial comms ");
  lcd.printByte(0); // clock

  Serial.begin(9600);
  // wait for serial monitor to open
  while (!Serial);
  lcd.clear();


  // connect to io.adafruit.com
  Serial.print("Connecting to Adafruit IO");
  lcd.print("Joining Ada IO ");
  lcd.printByte(0); // clock
  lcd.setCursor(0, 1);
  io.connect();
  // wait for a connection
  while (io.status() < AIO_CONNECTED)
  {
    Serial.print(".");
    lcd.print(".");
    delay(500);
  }
  lcd.clear();

  // we are connected
  Serial.println(io.statusText());
  lcd.clear();
  lcd.home();
  lcd.print("Connected!");

  // WiFi.mode(WIFI_STA);
  // ThingSpeak.begin(client);  // Initialize ThingSpeak

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
  // io.run(); is required for all sketches.
  // it should always be present at the top of your loop
  // function. it keeps the client connected to
  // io.adafruit.com, and processes any incoming data.
  io.run();

  analTemp = get_analog_temp();
  padTemp = get_digital_temp();
  pwmValue = control_heat_pad(currentTemp);

  // send_iot_data();
  mouseFeed->save(analTemp);
  padFeed->save(padTemp);
  pwmFeed->save(pwmValue);


  // serial print values
  Serial.print(analTemp);
  Serial.print(" ");
  Serial.print(padTemp);
  Serial.print(" ");
  Serial.print(pwmValue);
  Serial.print(" ");
  Serial.println(targetTemperature);
  delay(5000);
}
