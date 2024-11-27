#include "iot_lib.h"
// define functions here

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void Iot::send_data(float analTemp, float padTemp, float pwmValue, float roomTemp, float ambTemp){
  if ((previousDataSend == 0) || (millis() - previousDataSend >= updateIOTInterval)) {
    // send actual data here
    mouseFeed->save(analTemp);
    padFeed->save(padTemp);
    pwmFeed->save(pwmValue);
    roomFeed->save(roomTemp);
    ambFeed->save(ambTemp);
    previousDataSend = millis();
  }
}


// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// check connections to Wifi and IOT, reconnect if required
void Iot::check_connection() {
  if (aio.status() == AIO_CONNECTED)
    aio.run();
  else
    aio.connect();
}
