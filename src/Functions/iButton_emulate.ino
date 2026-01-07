#include "OneWireHub.h" //ВЕРСИЯ 1.2.0 !!!
#include "DS2401.h"
#define iButtonPort 10

auto hub = OneWireHub(iButtonPort);
byte Key[8] = { 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x2F };
auto ds1990A = DS2401(Key[0], Key[1], Key[2], Key[3], Key[4], Key[5], Key[6]);

void setup() {
  hub.attach(ds1990A);
}

void loop() {
  hub.poll();
}