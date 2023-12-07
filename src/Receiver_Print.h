#include <esp_now.h>
#include <WiFi.h>

#define SIGNAL_TIMEOUT 20000  // This is signal timeout in milli seconds. We will reset the data if no signal

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

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  if (len == 0)
  {
    return;
  }
  memcpy(&receiverData, incomingData, sizeof(receiverData));

  char inputValuesString[100];
  sprintf(inputValuesString, 
          "%3d,%3d,%3d,%3d,%3d",
           receiverData.xAxisValue,
           receiverData.yAxisValue,
           receiverData.switch1Value,
           receiverData.switch2Value,
           receiverData.potValue);
  Serial.println(inputValuesString); 
      
  lastRecvTime = millis(); 
}

void setup() 
{
  Serial.begin(115200);
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
 


void loop()
{
  //Check Signal lost.
  unsigned long now = millis();
  if ( now - lastRecvTime > SIGNAL_TIMEOUT ) 
  {
    Serial.println("No Signal");  
  }
}