//
// VSON PM050 library for Arduino
// (c)2017, a.m.emelianov@gmail.com
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU License.
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU License V3 for more details.
//
// In case of ESP8266 You can to connect Serial to Alternative Serial pins (D7 RX, D8 TX).
// Warning! This can cause ESP hangs on reboot.
// Uncomment all Serial.swap() to try.
// As PM050 connected to UART RX/TX pins console debug is not very useful, but You can
// find PM1.0, PM2.5 and PM10 values somewhere within output.
//

#include <PM050.h>

#define MAX_LOOPS     50
#define LOOP_DELAY	  100

PM050<HardwareSerial> pm(&Serial);

void setup() {
  pm.begin();
  //Serial.swap();
}

void loop() {
  pm.read();
  if (pm.isUpdated()) {
  // If PMxx data changed print values
    //Serial.swap();
    Serial.println();
    Serial.println(pm.pm10);
    Serial.println(pm.pm25);
    Serial.println(pm.pm100);
    Serial.flush();
    //Serial.swap();
  }
  if (pm.noData() > MAX_LOOPS) {
  //If no valid data received during MAX_LOOPS queries reset Serial port
      pm.reset();
      //Serial.swap();
  }
  delay(LOOP_DELAY);	// Next run in LOOP_DELAY mSec
}