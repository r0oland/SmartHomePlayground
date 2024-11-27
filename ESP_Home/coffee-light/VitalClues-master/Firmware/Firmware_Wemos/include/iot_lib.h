#ifndef IOT_LIB
#define IOT_LIB

#include <Arduino.h> // always required when using platformio
#include <String.h>
#include "AdafruitIO_WiFi.h"
#include "..\include\secrets.h"

class Iot
{
  public:
    Iot(const char* userName, const char* key, const char* ssid, const char* pass, uint_fast32_t updateInt):
      aio(userName, key, ssid, pass){
        updateIOTInterval = updateInt;
        padFeed = aio.feed("padFeed");
        mouseFeed = aio.feed("mouseFeed");
        pwmFeed = aio.feed("pwmFeed");
        roomFeed = aio.feed("roomFeed");
        ambFeed = aio.feed("ambFeed");
      };
    inline ~Iot(){};

    inline void send_data(float analTemp, float padTemp, float pwmValue, float roomTemp, float ambTemp);
    inline void check_connection();

    AdafruitIO_WiFi aio; // handles connection to thing board
    AdafruitIO_Feed *padFeed;
    AdafruitIO_Feed *mouseFeed;
    AdafruitIO_Feed *pwmFeed;
    AdafruitIO_Feed *roomFeed;
    AdafruitIO_Feed *ambFeed;

    WiFiClient client; // handles connection to thing board
    uint_fast8_t wifiStatus = WL_IDLE_STATUS;
    uint_fast32_t lastSend;

  private:
    // we get 30 writes per minute
    uint_fast32_t updateIOTInterval;
    uint_fast32_t previousDataSend = updateIOTInterval;
};

#endif
