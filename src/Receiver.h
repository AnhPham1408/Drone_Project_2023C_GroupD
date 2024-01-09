#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

#define MAX_SIGNAL 2000 // Parameter required for the ESC definition
#define MIN_SIGNAL 1000 // Parameter required for the ESC definition

uint8_t receiverMacAddress[] = {0x48,0xE7,0x29,0x9F,0xDE,0x3C}; //48:E7:29:9F:DE:3C
unsigned long lastRecvTime = 0;
double base_yaw = anglez;

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
  float LatitudeValue;
  float LongtitudeValue;
  float AltitudeValue;
  int HourValue;
  int MinuteValue;
  int SecondValue;
  float gx;
  float gy;
  float gz;
  float ax;
  float ay;
  float az;
  int thrust;
  int flm;
  int frm;
  int rlm;
  int rrm;
  int jx;
  int jy;
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
  if (receiverData.switch1Value == 0 && receiverData.switch2Value == 1 && anglez > -165){
    base_yaw = anglez - 15;
    ref_yaw = base_yaw;
  } 
  else if (receiverData.switch1Value == 0 && receiverData.switch2Value == 1 && anglez <= -165 && anglez > -179){
    base_yaw = 179;
    ref_yaw = base_yaw;
  }
  else if(receiverData.switch2Value == 0 && receiverData.switch1Value == 1 && anglez < 165){
    base_yaw = anglez + 15;
    ref_yaw = base_yaw;
  }
  else if (receiverData.switch2Value == 0 && receiverData.switch1Value == 1 && anglez >= 165 && anglez < 179){
    base_yaw = -179;
    ref_yaw = base_yaw;
  }
  else if((receiverData.switch2Value == 1 && receiverData.switch1Value == 1)||(receiverData.switch2Value == 0 && receiverData.switch1Value == 0)){
  ref_yaw = base_yaw;
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
  
  esp_now_register_recv_cb(OnDataRecv);
}

void SendData(){
  gpsData.LatitudeValue = latitude;
  gpsData.LongtitudeValue = longtitude;
  gpsData.AltitudeValue = altitude;
  gpsData.HourValue = hour;
  gpsData.MinuteValue = minute;
  gpsData.SecondValue = second;
  gpsData.gx = gyrox;
  gpsData.gy = gyroy;
  gpsData.gz = gyroz;
  gpsData.ax = anglex;
  gpsData.ay = angley;
  gpsData.az = anglez;
  gpsData.thrust = ref_throttle;
  gpsData.flm = fl;
  gpsData.frm = fr;
  gpsData.rlm = rl;
  gpsData.rrm = rr;
  gpsData.jx = ref_roll;
  gpsData.jy = ref_pitch;
  esp_now_send(receiverMacAddress, (uint8_t *) &gpsData, sizeof(gpsData));
}