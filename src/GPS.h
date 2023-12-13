#include <Arduino.h>      // Arduino library
#include <TinyGPSPlus.h>

#define RXD2 16
#define TXD2 17
TinyGPSPlus gps;

int latitude, longtitude, altitude, hour, minute, second;

void displayInfo();

void Get_GPSData(){
  while (Serial2.available() > 0)
    if (gps.encode(Serial2.read())){
      if (gps.location.isValid()){
        latitude = gps.location.lat();
        longtitude = gps.location.lng();
      }
      if (gps.altitude.isValid()){
        altitude = gps.altitude.meters();
      }
      if (gps.date.isValid()){
        hour = gps.time.hour();
        minute = gps.time.minute();
        second = gps.time.second();
      }
    }
}