#include <Arduino.h>      // Arduino library
#include <ESP32Servo.h> //Use the Servo librarey for generating PWM
#include "PID_v1.h"        // Library to configure the PID
//#include "Receiver.h"

// ================================================================
// Variable declaration

Servo FLESC; //name the servo object, here ESC 
Servo FRESC;
Servo RLESC;
Servo RRESC;

const int flEscPin = 33;
const int frEscPin = 32;
const int rlEscPin = 25;
const int rrEscPin = 26;

extern double   ref_throttle,
                ref_yaw,
                ref_pitch,
                ref_roll; //Reference control value
            
int fl,fr,rl,rr;

//PID constants
//kp_roll = 1.9, kd_roll = 15, ki_roll = 0.015
double  kp_roll_gy = 5,
        kd_roll_gy = 0.03,                                                                                  
        ki_roll_gy = 0.22,  
        kp_roll_an = 0.85,     
        kd_roll_an = 0.03,
        ki_roll_an = 0;
        
double  kp_pitch_gy = 5,
        kd_pitch_gy = 0.032, 
        ki_pitch_gy = 0.22,
        kp_pitch_an = 0.85,
        kd_pitch_an = 0.032,
        ki_pitch_an = 0;

double  kp_yaw_gy = 3,
        kd_yaw_gy = 0,
        ki_yaw_gy = 0,
        kp_yaw_an = 3,
        kd_yaw_an = 0,
        ki_yaw_an = 0;

double ang_roll = 0,  ang_pitch = 0, ang_yaw = 0; // PID output outter loop
double u_roll = 0, u_pitch = 0, u_yaw = 0;; // PID output inner loop
const int SAMPLE_TIME = 10;

PID PID_ax(&anglex, &ang_roll, &ref_roll, kp_roll_an, ki_roll_an, kd_roll_an, DIRECT);
PID PID_gx(&gyrox, &u_roll, &ang_roll, kp_roll_gy, ki_roll_gy, kd_roll_gy, DIRECT);
PID PID_ay(&angley, &ang_pitch, &ref_pitch, kp_pitch_an, ki_pitch_an, kd_pitch_an, DIRECT);
PID PID_gy(&gyroy, &u_pitch, &ang_pitch, kp_pitch_gy, ki_pitch_gy, kd_pitch_gy, DIRECT);
PID PID_az(&anglez, &ang_yaw, &ref_yaw, kp_yaw_an, ki_yaw_an, kd_yaw_an, DIRECT);
PID PID_gz(&gyroz, &u_yaw, &ang_yaw, kp_yaw_gy, ki_yaw_gy, kd_yaw_gy, DIRECT);

void Init_PID(){
    FLESC.attach(flEscPin); //Generate PWM 
    FRESC.attach(frEscPin);
    RLESC.attach(rlEscPin);
    RRESC.attach(rrEscPin);
    PID_ax.SetMode(AUTOMATIC);
    PID_ax.SetOutputLimits(-127, 127);
    PID_ax.SetSampleTime(SAMPLE_TIME);
    PID_gx.SetMode(AUTOMATIC);
    PID_gx.SetOutputLimits(-127, 127);
    PID_gx.SetSampleTime(SAMPLE_TIME);
    PID_ay.SetMode(AUTOMATIC);
    PID_ay.SetOutputLimits(-127, 127);
    PID_ay.SetSampleTime(SAMPLE_TIME);
    PID_gy.SetMode(AUTOMATIC);
    PID_gy.SetOutputLimits(-127, 127);
    PID_gy.SetSampleTime(SAMPLE_TIME);
    PID_az.SetMode(AUTOMATIC);
    PID_az.SetOutputLimits(-127, 127);
    PID_az.SetSampleTime(SAMPLE_TIME);
    PID_gz.SetMode(AUTOMATIC);
    PID_gz.SetOutputLimits(-127, 127);
    PID_gz.SetSampleTime(SAMPLE_TIME);
}
void Compute_PID(){
    //Roll Calculation
    PID_ax.SetTunings(kp_roll_an, ki_roll_an, kd_roll_an);
    PID_ax.Compute();
    PID_gx.SetTunings(kp_roll_gy, ki_roll_gy, kd_roll_gy);
    PID_gx.Compute();
    //Pitch Calculation
    PID_ay.SetTunings(kp_pitch_an, ki_pitch_an, kd_pitch_an);
    PID_ay.Compute();
    PID_gy.SetTunings(kp_pitch_gy, ki_pitch_gy, kd_pitch_gy);
    PID_gy.Compute();
    //Yaw calculation
    PID_az.SetTunings(kp_yaw_an, ki_yaw_an, kd_yaw_an);
    PID_az.Compute();
    PID_gz.SetTunings(kp_yaw_gy, ki_yaw_gy, kd_yaw_gy);
    PID_gz.Compute();
}

void updateMotor(){
    double u_throttle = ref_throttle;
    Compute_PID();
    fl = u_throttle + u_roll - u_pitch - u_yaw;
    fr = u_throttle - u_roll - u_pitch + u_yaw;
    rl = u_throttle + u_roll + u_pitch + u_yaw;
    rr = u_throttle - u_roll + u_pitch - u_yaw;
    
    // Define maximum control signal to prevent motor burn out
    if (fl >= 2000)
      fl = 2000;
    if (fr >= 2000)
      fr = 2000;
    if (rl >= 2000)
      rl = 2000;
    if (rr >= 2000)
      rr = 2000;
    // Make 4 motors spin up same time
    if(ref_throttle <= 1100){
      fl = 1000;
      fr = 1000;
      rl = 1000;
      rr = 1000;
    }
    // Motor turn of if roll or pitch angle over 50 degrees
    if(abs(anglex) >= 50 || abs(angley) >= 50) {
      fl = 1000;
      fr = 1000;
      rl = 1000;
      rr = 1000;
    }
    //write
    FLESC.writeMicroseconds(fl);
    FRESC.writeMicroseconds(fr);
    RLESC.writeMicroseconds(rl);
    RRESC.writeMicroseconds(rr);
}
/*
fl = + - +
fr = - - -
rl = + + -
rr = - + +
*/