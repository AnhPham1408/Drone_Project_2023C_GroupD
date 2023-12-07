#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

#define SIGNAL_TIMEOUT 1000  // This is signal timeout in milli seconds. We will reset the data if no signal
#define MAX_SIGNAL 2000 // Parameter required for the ESC definition
#define MIN_SIGNAL 1000 // Parameter required for the ESC definition

unsigned long lastRecvTime = 0;

struct PacketData
{
  byte xAxisValue;
  byte yAxisValue;
 
  byte switch1Value;
  byte switch2Value;

  byte potValue;
};
PacketData receiverData;

Servo ESC1;     
Servo ESC2;     
Servo ESC3;     
Servo ESC4;     

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

void mapAndWriteValues()
{
  ESC1.write(receiverData.potValue);
  ESC2.write(receiverData.potValue);
  ESC3.write(receiverData.potValue);
  ESC4.write(receiverData.potValue);
  
  //digitalWrite(led1, receiverData.switch1Value);

}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  if (len == 0)
  {
    return;
  }
  memcpy(&receiverData, incomingData, sizeof(receiverData));
  mapAndWriteValues();  
  lastRecvTime = millis(); 
}

void setUpPinModes()
{
  ESC1.attach(27, MIN_SIGNAL, MAX_SIGNAL);
  ESC1.writeMicroseconds(MIN_SIGNAL);
  ESC2.attach(26, MIN_SIGNAL, MAX_SIGNAL);
  ESC2.writeMicroseconds(MIN_SIGNAL);
  ESC3.attach(25, MIN_SIGNAL, MAX_SIGNAL);
  ESC3.writeMicroseconds(MIN_SIGNAL);
  ESC4.attach(33, MIN_SIGNAL, MAX_SIGNAL);
  ESC4.writeMicroseconds(MIN_SIGNAL);

  //pinMode(led1, OUTPUT);
  
  setInputDefaultValues();
  mapAndWriteValues();
}

void setup() 
{
  setUpPinModes();
 
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  Serial.print("ESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());

  esp_now_register_recv_cb(OnDataRecv);
}
 


void loop()
{
  //Check Signal lost.
  unsigned long now = millis();
  if ( now - lastRecvTime > SIGNAL_TIMEOUT ) 
  {
    setInputDefaultValues();
    mapAndWriteValues();  
  }
}