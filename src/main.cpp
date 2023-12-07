#include <Arduino.h>      // Arduino library
#include "MPU.h"          // Personal library to configure the MPU6050
#include <TinyGPSPlus.h>
#include <ESP32Servo.h> //Use the Servo librarey for generating PWM
#include <esp_now.h>
#include <WiFi.h>
//#include "PID.h"        // Personnal library to configure the PID

// ================================================================
// Variable declaration
unsigned long time_prev = 0;
unsigned long lastRecvTime = 0;
#define SIGNAL_TIMEOUT 1000  // This is signal timeout in milli seconds. We will reset the data if no signal
#define RXD2 16
#define TXD2 17
TinyGPSPlus gps;

Servo FLESC; //name the servo object, here ESC 
Servo FRESC;
Servo RLESC;
Servo RRESC;

const int flEscPin = 25;
const int frEscPin = 26;
const int rlEscPin = 32;
const int rrEscPin = 33;

// current rotational rate in degrees/second
float gyro_pitch, gyro_roll, gyro_yaw;
float curr_roll, curr_pitch;

int controller_active;

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

int fl,fr,rl,rr;

float ref_throttle,
      ref_yaw,
      ref_pitch,
      ref_roll;

//PID constants
//kp_roll = 1.9, kd_roll = 15, ki_roll = 0.015
const float kp_roll = 0.6,
            kd_roll = 0.01,
            ki_roll = 3.5,
            kp_roll_an = 0.5,
            kd_roll_an = 0,
            ki_roll_an = 0;

const float kp_pitch = 0.6,
            kd_pitch = 0.01,
            ki_pitch = 3.5,
            kp_pitch_an = 0.5,
            kd_pitch_an = 0,
            ki_pitch_an = 0;

const float kp_yaw = 2,
            kd_yaw = 0,
            ki_yaw = 12;

// derivative and integral errors
float prev_err_roll = 0,
      eint_roll = 0,
      prev_err_roll_angle = 0,
      eint_roll_angle = 0;

float prev_err_pitch = 0,
      eint_pitch = 0,
      prev_err_pitch_angle = 0,
      eint_pitch_angle = 0;

float prev_err_yaw = 0,
      eint_yaw = 0;
      
// ================================================================
// Most of the variables are declared in the personal library
// ================================================================
// Function Declaration
// ================================================================
// These function are kept in the main.cpp because it is easier to modify
void Init_Serial();
void SerialDataPrint();
void Get_GPSData();
void setInputDefaultValues();
void displayInfo();
void readJoystick();;
void updateMotor();

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
// ================================================================
// Setup function
// ================================================================
void setup(){
  Init_MPU();       // Initialize the MPU
  Init_Serial();    // Initialize the GPS
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  FLESC.attach(flEscPin); //Generate PWM 
  FRESC.attach(frEscPin);
  RLESC.attach(rlEscPin);
  RRESC.attach(rrEscPin);

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
  //Get_GPSData();     // Get the GPS data 
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
// ================================================================
void setInputDefaultValues()
{
  // The middle position for joystick. (254/2=127)
  receiverData.xAxisValue = 127;
  receiverData.yAxisValue = 127;
  
  receiverData.switch1Value = LOW;
  receiverData.switch2Value = LOW;

  receiverData.potValue = 0;
}
// ================================================================
void readJoystick() {
  ref_throttle = map(receiverData.potValue, 0, 180, 1000, 1900);
  if (receiverData.switch1Value == 0 && receiverData.switch2Value == 1){
    ref_yaw = -5;
  } 
  else if(receiverData.switch2Value == 0 && receiverData.switch1Value == 1){
    ref_yaw = 5;
  }
  else if((receiverData.switch2Value == 1 && receiverData.switch1Value == 1)||(receiverData.switch2Value == 0 && receiverData.switch1Value == 0)){
  ref_yaw = 0;
  }
  ref_pitch = map(receiverData.yAxisValue, 0, 254, -10, 10);;
  ref_roll = map(receiverData.xAxisValue, 0, 254, -10, 10);;
}
// ================================================================
void updateMotor() {
  if (controller_active) {
    float u_throttle = ref_throttle;
    gyro_roll = gyrox;
    gyro_pitch = gyroy;
    gyro_yaw = gyroz;
    curr_roll = anglex;
    curr_pitch = angley;

    // do calculation **ROLL**
    float err_roll_angle = 0;
    float edot_roll_angle = 0;
    float ang_roll = 0;

    err_roll_angle = ref_roll - curr_roll;
    edot_roll_angle = err_roll_angle - prev_err_roll_angle;
    eint_roll_angle = eint_roll_angle + err_roll_angle;

    ang_roll = (kp_roll_an * err_roll_angle) + (kd_roll_an * edot_roll_angle) + (ki_roll_an * eint_roll_angle);

    float err_roll = 0;
    float edot_roll = 0;
    float u_roll = 0;
    
    err_roll = ang_roll - gyro_roll;
    edot_roll = err_roll - prev_err_roll;
    eint_roll = eint_roll + err_roll;

    u_roll = (kp_roll * err_roll) + (kd_roll * edot_roll) + (ki_roll * eint_roll);

    // do calculation **PITCH**
    float err_pitch_angle = 0;
    float edot_pitch_angle = 0;
    float ang_pitch = 0;

    err_pitch_angle = ref_pitch - curr_pitch;
    edot_pitch_angle = err_pitch_angle - prev_err_pitch_angle;
    eint_pitch_angle = eint_pitch_angle + err_pitch_angle;

    ang_pitch = (kp_pitch_an * err_pitch_angle) + (kd_pitch_an * edot_pitch_angle) + (ki_pitch_an * eint_pitch_angle);

    float err_pitch = 0;
    float edot_pitch = 0;
    float u_pitch = 0;

    err_pitch = ang_pitch - gyro_pitch;
    edot_pitch = err_pitch - prev_err_pitch;
    eint_pitch = eint_pitch + err_pitch;

    u_pitch = (kp_pitch * err_pitch) + (kd_pitch * edot_pitch) + (ki_pitch * eint_pitch);

    // do calculation **YAW**
    float err_yaw= 0;
    float edot_yaw = 0;
    float u_yaw = 0;
    //float curr_yaw = anglez;

    err_yaw = ref_yaw - gyro_yaw;
    edot_yaw = err_yaw - prev_err_yaw;
    eint_yaw = eint_yaw + err_yaw;
    
    u_yaw = (kp_yaw * err_yaw) + (kd_yaw * edot_yaw) + (ki_yaw * eint_yaw);


    fl = u_throttle + u_roll + u_pitch - u_yaw;
    fr = u_throttle - u_roll + u_pitch + u_yaw;
    rl = u_throttle + u_roll - u_pitch + u_yaw;
    rr = u_throttle - u_roll - u_pitch - u_yaw;

    // if motor output gets too high reset the integral error to 0.
    // This is to prevent the integral term from getting too high.
    if ((fl >= 2000) || (fr >= 2000) || (rl >= 2000) || (rr >= 2000)) {
      eint_roll = 0;
      eint_pitch = 0;
      eint_roll_angle = 0;
      eint_pitch_angle = 0;
      eint_yaw = 0;
    }

    //Serial.print("Gyro_yaw: "); Serial.print(gyro_yaw);
    //Serial.print(" target_yaw: "); Serial.println(ref_yaw);
    //Serial.print(" error: "); Serial.println(err_pitch);

    /*
    Serial.print("fl: "); Serial.print(fl);
    Serial.print(" fr: "); Serial.print(fr);
    Serial.print(" rl: "); Serial.print(rl);
    Serial.print(" rr: "); Serial.println(rr);
    
      // update new output signal
      int fl = u_throttle + u_yaw + u_pitch - u_roll;
      int fr = u_throttle - u_yaw + u_pitch + u_roll;
      int rl = u_throttle - u_yaw - u_pitch - u_roll;
      int rr = u_throttle + u_yaw - u_pitch + u_roll;
    */

    //write
    FLESC.writeMicroseconds(fl);
    FRESC.writeMicroseconds(fr);
    RLESC.writeMicroseconds(rl);
    RRESC.writeMicroseconds(rr);

    // current error becomes the previous error for next cycle
    prev_err_roll = err_roll;
    prev_err_pitch = err_pitch;
    prev_err_yaw = err_yaw;
    prev_err_roll_angle = err_roll_angle;
    prev_err_pitch_angle = err_pitch_angle;
  
  }
  else {
    //Serial.println("Disamred. Shutting off motors..");
    FLESC.writeMicroseconds(0);
    FRESC.writeMicroseconds(0);
    RLESC.writeMicroseconds(0);
    RRESC.writeMicroseconds(0);
    eint_roll = 0;
    eint_pitch = 0;
    eint_yaw = 0;
  }

}
// ================================================================
void Get_GPSData()
{
  while (Serial2.available() > 0)
    if (gps.encode(Serial2.read()))
      displayInfo();
}
// ================================================================
void displayInfo()
{
  Serial.print(F("Location: "));
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Altitude: "));
  if (gps.altitude.isValid())
  {
    Serial.print(gps.altitude.meters());
    Serial.print(F("m"));
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }
  Serial.println();
}
