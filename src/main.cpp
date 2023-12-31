#include <Arduino.h>      // Arduino library
#include "MPU.h"          // Personal library to configure the MPU6050
#include "GPS.h"
#include "PID.h"
#include "Receiver.h"
#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h> //Use the Servo librarey for generating PWM
// ================================================================
// Variable declaration
unsigned long time_prev = 0;
#define SIGNAL_TIMEOUT 1000  // This is signal timeout in milli seconds. We will reset the data if no signal
#define DATA_STIME 50 //The time between each data send cycle
#define DATA_PTIME 1000 //The time between each data send cycle
bool signal_send;
int cnt;
int controller_active;
// ================================================================
// These function are kept in the main.cpp because it is easier to modify
void FlipFlop();
void Init_Serial();
void SerialDataPrint();
// ================================================================
// Setup function
void setup(){
  Init_MPU();       // Initialize the MPU
  Init_Serial();    // Initialize the GPS
  Init_PID();
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  WiFi.mode(WIFI_STA);
  //Serial.print("ESP Board MAC Address:  ");
  //Serial.println(WiFi.macAddress());
  ESPnowInit();
}
// ================================================================
// Loop function
void loop(){
  controller_active = true; 
  Get_GPSData();
  Get_MPUangle();    // Get the angle (angle) from the IMU sensor
  Get_accelgyro();
  FlipFlop();
  unsigned long now = millis();
  if ( now - lastRecvTime > SIGNAL_TIMEOUT ) {
    // reset reference value to 0 at the start of every loop. This is to prevent
    // motors from spinning when it loses connection.
    Serial.println("Error Receiving the data");
    setInputDefaultValues();
    controller_active = false;
  }
  updateMotor();
  if (signal_send){
    SendData();
  }
  SerialDataPrint(); // Print the data on the serial monitor for debugging
}

// ================================================================
// Function Definition
//Create flip flops to delay sending time
void FlipFlop() {
  if (cnt == DATA_STIME){
    cnt = 0;
    signal_send = true;
  }
  else{
    cnt = cnt + 1;
    signal_send = false;
  }
}
// ================================================================
void SerialDataPrint()
{
  if (millis() - time_prev >= DATA_PTIME)
  {
    time_prev = millis(); 
    Serial.print(anglex);
    Serial.print("\t");
    Serial.print(angley);
    Serial.print("\t");
    Serial.print(anglez);
    Serial.print("\t");
    Serial.print(ref_throttle);
    Serial.print("\tz");
    Serial.print("fl: ");   Serial.print(fl);    
    Serial.print("\t");
    Serial.print("fr: ");   Serial.print(fr);    
    Serial.print("\t");
    Serial.print("rl: ");   Serial.print(rl);    
    Serial.print("\t");
    Serial.print("rr: ");   Serial.print(rr);    
    Serial.print("\t");
    Serial.print(gyrox);
    Serial.print("\t");
    Serial.print(gyroy);
    Serial.print("\t");
    Serial.print(gyroz);
    Serial.print("\t");
    Serial.print(millis());
    Serial.print("\t");
    Serial.print(latitude);
    Serial.print("\t");
    Serial.print(longtitude);
    Serial.print("\t");
    Serial.print(altitude);
    Serial.print("\t");
    Serial.println();   
  }
}
// ================================================================
void Init_Serial()
{
  Serial.begin(115200);
  while (!Serial){
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(0x68);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  };
}