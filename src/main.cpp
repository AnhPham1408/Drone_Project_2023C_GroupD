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
int controller_active;
      
// ================================================================
// Most of the variables are declared in the personal library
// ================================================================
// Function Declaration
// ================================================================
// These function are kept in the main.cpp because it is easier to modify
void Init_Serial();
void SerialDataPrint();
// ================================================================
// Setup function
// ================================================================
void setup(){
  Init_MPU();       // Initialize the MPU
  Init_Serial();    // Initialize the GPS
  Init_PID();

  WiFi.mode(WIFI_STA);
  //Serial.print("ESP Board MAC Address:  ");
  //Serial.println(WiFi.macAddress());
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  esp_now_register_recv_cb(OnDataRecv);
}
// ================================================================
// Loop function
// ================================================================
void loop(){
  controller_active = true;
  Get_MPUangle();    // Get the angle (angle) from the IMU sensor
  Get_accelgyro();
    unsigned long now = millis();
  if ( now - lastRecvTime > SIGNAL_TIMEOUT ) 
  {
    setInputDefaultValues();
    // reset reference value to 0 at the start of every loop. This is to prevent
    // motors from spinning when it loses connection.
    ref_throttle = 0;
    ref_yaw = 0;
    ref_pitch = 0;
    ref_roll = 0;
    controller_active = false;
  }
  updateMotor();
  SerialDataPrint(); // Print the data on the serial monitor for debugging
}

// ================================================================
// Function Definition
// ================================================================
void SerialDataPrint()
{
  if (micros() - time_prev >= 50000)
  {
    time_prev = micros(); 
    Serial.print(anglex);
    Serial.print("\t");
    Serial.print(angley);
    Serial.print("\t");
    Serial.print(anglez);
    Serial.print("\t");
    Serial.print(ref_throttle);
    Serial.print("\tz");
    Serial.print("fl: ");   Serial.print(fl);    Serial.print("\t");
    Serial.print("fr: ");   Serial.print(fr);    Serial.print("\t");
    Serial.print("rl: ");   Serial.print(rl);    Serial.print("\t");
    Serial.print("rr: ");   Serial.print(rr);    Serial.print("\t");
    Serial.print(gyrox);
    Serial.print("\t");
    Serial.print(gyroy);
    Serial.print("\t");
    Serial.print(gyroz);
    Serial.print("\t");
    Serial.print(millis());
    Serial.print("\t");
    /*
    Serial.print(motor_cmd);
    Serial.print("\t");
    Serial.print(kp);
    Serial.print("\t");
    Serial.print(ki);
    Serial.print("\t");
    Serial.print(kd);
    Serial.print("\t");
    Serial.print(anglex_setpoint); */
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