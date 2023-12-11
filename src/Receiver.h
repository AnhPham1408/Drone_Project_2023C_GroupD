#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

#define SIGNAL_TIMEOUT 1000  // This is signal timeout in milli seconds. We will reset the data if no signal
#define MAX_SIGNAL 2000 // Parameter required for the ESC definition
#define MIN_SIGNAL 1000 // Parameter required for the ESC definition

unsigned long lastRecvTime = 0;

double  ref_throttle,
        ref_yaw,
        ref_pitch,
        ref_roll; //Reference control value

// Reference values received from the server
struct PacketData
{
  byte xAxisValue;
  byte yAxisValue;
 
  byte switch1Value;
  byte switch2Value;

  byte potValue;
};
PacketData receiverData;

//Assign default input received values
void setInputDefaultValues()
{
  // The middle position for joystick. (254/2=127)
  receiverData.xAxisValue = 127;
  receiverData.yAxisValue = 127;
  
  receiverData.switch1Value = LOW;
  receiverData.switch2Value = LOW;

  receiverData.potValue = 0;
}

void readJoystick() {
  ref_throttle = map(receiverData.potValue, 0, 180, 1000, 1900);
  if (receiverData.switch1Value == 0 && receiverData.switch2Value == 1){
    ref_yaw = -20;
  } 
  else if(receiverData.switch2Value == 0 && receiverData.switch1Value == 1){
    ref_yaw = 20;
  }
  else if((receiverData.switch2Value == 1 && receiverData.switch1Value == 1)||(receiverData.switch2Value == 0 && receiverData.switch1Value == 0)){
  ref_yaw = 0;
  }
  ref_pitch = map(receiverData.yAxisValue, 0, 254, -10, 10);;
  ref_roll = map(receiverData.xAxisValue, 0, 254, -10, 10);;
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  if (len == 0)
  {
    return;
  }
  memcpy(&receiverData, incomingData, sizeof(receiverData));
  readJoystick();
      
  lastRecvTime = millis(); 
}