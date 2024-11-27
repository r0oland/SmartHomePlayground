#include "..\lib\vital_lib.cpp"
#include "..\lib\iot_lib.cpp"
#include "..\include\secrets.h"

uint_fast32_t updateIOTInterval = 10000;

Iot MyIot(IO_USERNAME,IO_KEY,WIFI_SSID,WIFI_PASS,updateIOTInterval);
Vital MyVital(ONE_WIRE_BUS);


DeviceAddress PAD_SENS  = {0x28, 0xFF, 0x67, 0x98, 0x90, 0x16, 0x04, 0xFA};
DeviceAddress ROOM_SENS = {0x28, 0x70, 0x56, 0x79, 0xA2, 0x00, 0x03, 0xEF};
DeviceAddress AMB_SENS  = {0x28, 0xF9, 0x9D, 0x79, 0xA2, 0x00, 0x03, 0xC1};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void setup(void) {
  setup_serial();

  MyVital.setup_screen();

  MyVital.screen_print("Setting up pins..."); 
  MyVital.setup_io_pins();
  MyVital.screen_println("done!"); 

  MyVital.screen_print("Setting up sensors..."); 
  MyVital.setup_temp_sensor(12); // set resolution for temp sensor
  MyVital.PadTempSens =     &PAD_SENS;
  MyVital.RoomTempSens =    &ROOM_SENS;
  MyVital.AmbientTempSens = &AMB_SENS;
  MyVital.get_temp_sensor_address();
  MyVital.screen_println("done!"); 

  MyVital.screen_println("IOT connecting:"); 
  MyIot.aio.connect();
  // wait for a connection
  while (MyIot.aio.status() < AIO_CONNECTED)
  {
    MyVital.screen_print(".");
    delay(500);
  }
  MyVital.screen_println("connected!");

  MyVital.screen_print("Finding time server.");
  configTime(TIME_ZONE*3600, 0 , "pool.ntp.org", "time.nist.gov"); // 
  delay(500);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void loop(void) {
  MyIot.check_connection();
  MyVital.control_heat_pads();

  MyIot.send_data(MyVital.analTemp, MyVital.padTemp, MyVital.pwmValue, MyVital.roomTemp, MyVital.ambTemp);

  bool aioConnected = (MyIot.aio.status() == AIO_CONNECTED);
  MyVital.update_screen(aioConnected);
  MyVital.update_serial();
}



