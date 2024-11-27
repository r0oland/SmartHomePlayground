#ifndef VITAL_LIB
#define VITAL_LIB

#include <Arduino.h> // always required when using platformio
#include <ESP8266WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <U8g2lib.h> 
#include <time.h>

#include "..\include\secrets.h"

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Pin definitions
const uint8_t ONE_WIRE_BUS = D3; // temp. sensor connected on D2
const uint8_t MOUSE_PAD_PIN = D4; // transistor controll pin
const uint8_t MOUSE_ROOM_PIN = D5; // transistor controll pin
const uint8_t THERMISTORPIN = A0;
const uint8_t I2C_SCL_PIN = D1;
const uint8_t I2C_SDA_PIN = D2;
#define SCREEN_ROT U8G2_R0 // no screen rotation

// % screen related settings------------------------------------------
const uint16_t FRAME_RATE = 3; // frames per second
const uint16_t FRAME_TIME_MILLIS = 1000./FRAME_RATE; // time per frame in milis.
const uint8_t LINE_SPACING = 8; // we can print an new, non-overlapping line
const int8_t TIME_ZONE = +2; // central european time


// General Function Declarations
void setup_serial();

// Vital Class Definition %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
class Vital
{
  public:
    Vital(uint8_t oneWireBus):
      V_OneWire(ONE_WIRE_BUS),
      V_Screen(SCREEN_ROT,I2C_SCL_PIN,I2C_SDA_PIN),
      V_TempSensor(&V_OneWire)
      {
        for (iErr = 0; iErr < nInt; iErr++)  // reset error container to 0s
          errContainer[iErr] = 0;
        iErr = 0;
      };

    void setup_screen();
    void setup_io_pins();
    void setup_temp_sensor(uint8_t sensReso);
    void get_temp_sensor_address();
    void update_serial();
    void get_analog_temp();
    void get_digital_temp();
    void control_heat_pads();
    void update_screen(bool iotConnected);
    void screen_go_next_line();
    void screen_println(String printStr, bool updateBuffer = true);
    void screen_print(String printStr, bool updateBuffer = true);

    U8G2_SSD1306_128X64_NONAME_F_SW_I2C V_Screen;
    OneWire V_OneWire;
    DallasTemperature V_TempSensor;
    DeviceAddress* PadTempSens;
    DeviceAddress* RoomTempSens;
    DeviceAddress* AmbientTempSens;

    // Vars for screen control
    uint8_t currentLine = 0; // line we are currently printing to

    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // variables for analog & digital temperature measurements
    const uint16_t THERMISTORNOMINAL = 10000;
    const float TEMPERATURENOMINAL = 25.0;
    const uint8_t N_SAMPLES = 10;
    const uint16_t BCOEFFICIENT = 3960;
    const uint16_t SERIESRESISTOR = 9970;
    const float faultTemperature = -120; // TODO make sure to use this!

    float targetTemperature = 37.0;
    float maxPadTemp = 45.0;
    float analTemp = 0.0; // temperature measured in anus
    float padTemp = 0.0;
    float roomTemp = 0.0;
    float ambTemp = 0.0;

    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // PID variables
    float err; // difference between measured and set point
    float previous_err = 0;
    float integral = 0;
    float derivative = 0;
    const uint_fast8_t nInt = 20;
    float errContainer[20];
    unsigned char iErr; // runs from 1 to 10
    const float pValue = 90.0; // proportional part of controller
    const float dValue = 0; // differential part of controller
    const float iValue = 10; // integral part of controller
    float pwmValue = 0; // set initial heating to 0
    int iPwmValue = 0; // inted pwm value

  private:
};


#endif
