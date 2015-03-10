#include <Movi.h>
#include <XBee.h>

XBee xbee = XBee();
Movi movi = Movi(NORMAL);
ZBRxResponse rx = ZBRxResponse();

void setup() {
  Serial.begin(57600);
  xbee.begin(Serial);
  movi.initPin();
}

void loop() {
  
    xbee.readPacket();
    if (xbee.getResponse().isAvailable()) {
      if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
        xbee.getResponse().getZBRxResponse(rx);
        movi.automov(rx.getData(0),rx.getData(1),rx.getData(2),rx.getData(3));
      }
  }
}

