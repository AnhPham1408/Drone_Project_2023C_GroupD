#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

#define SIGNAL_TIMEOUT 1000  // This is signal timeout in milli seconds. We will reset the data if no signal
#define MAX_SIGNAL 2000 // Parameter required for the ESC definition
#define MIN_SIGNAL 1000 // Parameter required for the ESC definition

uint8_t receiverMacAddress[] = {0x48,0xE7,0x29,0x9F,0xDE,0x3C}; //48:E7:29:9F:DE:3C
unsigned long lastRecvTime = 0;

double  ref_throttle,
        ref_yaw,
        ref_pitch,
        ref_roll; //Reference control value

// Reference values received from the server
struct PacketData{
  byte xAxisValue;
  byte yAxisValue;
 
  byte switch1Value;
  byte switch2Value;

  byte potValue;
};
PacketData receiverData;

struct GPSData{
  uint16_t LatitudeValue;
  uint16_t LongtitudeValue;
  uint16_t AltitudeValue;
  byte HourValue;
  byte MinuteValue;
  byte SecondValue;
  byte sign;
  uint16_t ax;
  uint16_t ay;
  uint16_t az;
  uint16_t thrust;
  uint16_t flm;
  uint16_t frm;
  uint16_t rlm;
  uint16_t rrm;
};
GPSData gpsData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  // Serial.print("\r\nLast Packet Send Status:\t ");
  // Serial.println(status);
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Message sent" : "Message failed");
}

void ESPnowInit(){
      // Init ESP-NOW
  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  else
  {
    Serial.println("Succes: Initialized ESP-NOW");
  }

  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
  else
  {
    Serial.println("Succes: Added peer");
  } 
}

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

void SendData(){
  gpsData.LatitudeValue = latitude;
  gpsData.LongtitudeValue = longtitude;
  gpsData.AltitudeValue = altitude;
  gpsData.HourValue = hour;
  gpsData.MinuteValue = minute;
  gpsData.SecondValue = second;

  if (anglex < 0 && angley < 0 && anglez < 0){
    gpsData.sign = 0;
    gpsData.ax = floor(anglex*-100);
    gpsData.ay = floor(angley*-100);
    gpsData.az = floor(anglez*-100);
  }
  else if (anglex > 0 && angley < 0 && anglez < 0){ 
    gpsData.sign = 1;
    gpsData.ax = floor(anglex*100);
    gpsData.ay = floor(angley*-100);
    gpsData.az = floor(anglez*-100);
  }
  else if (anglex < 0 && angley > 0 && anglez < 0){ 
    gpsData.sign = 2;
    gpsData.ax = floor(anglex*-100);
    gpsData.ay = floor(angley*100);
    gpsData.az = floor(anglez*-100);
  }
  else if (anglex < 0 && angley < 0 && anglez > 0){ 
    gpsData.sign = 3;
    gpsData.ax = floor(anglex*-100);
    gpsData.ay = floor(angley*-100);
    gpsData.az = floor(anglez*100);
  }
  else if (anglex > 0 && angley > 0 && anglez < 0){ 
    gpsData.sign = 4;
    gpsData.ax = floor(anglex*100);
    gpsData.ay = floor(angley*100);
    gpsData.az = floor(anglez*-100);
  }
  else if (anglex > 0 && angley < 0 && anglez > 0){ 
    gpsData.sign = 5;
    gpsData.ax = floor(anglex*100);
    gpsData.ay = floor(angley*-100);
    gpsData.az = floor(anglez*100);
  }
  else if (anglex < 0 && angley > 0 && anglez > 0){ 
    gpsData.sign = 6;
    gpsData.ax = floor(anglex*-100);
    gpsData.ay = floor(angley*100);
    gpsData.az = floor(anglez*100);
  }
  else{
    gpsData.sign = 7;
    gpsData.ax = floor(anglex*100);
    gpsData.ay = floor(angley*100);
    gpsData.az = floor(anglez*100);
  }

  gpsData.thrust = ref_throttle;
  gpsData.flm = fl;
  gpsData.frm = fr;
  gpsData.rlm = rl;
  gpsData.rrm = rr;
  esp_now_send(receiverMacAddress, (uint8_t *) &gpsData, sizeof(gpsData));
}