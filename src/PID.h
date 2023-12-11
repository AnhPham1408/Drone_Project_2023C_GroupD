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

const int flEscPin = 25;
const int frEscPin = 26;
const int rlEscPin = 32;
const int rrEscPin = 33;

extern double   ref_throttle,
                ref_yaw,
                ref_pitch,
                ref_roll; //Reference control value
            
int fl,fr,rl,rr;

//PID constants
//kp_roll = 1.9, kd_roll = 15, ki_roll = 0.015
double  kp_roll = 0.6,
        kd_roll = 0.01,
        ki_roll = 3.5,
        kp_roll_an = 0.5,
        kd_roll_an = 0,
        ki_roll_an = 0;
        
double  kp_pitch = 0.6,
        kd_pitch = 0.01,
        ki_pitch = 3.5,
        kp_pitch_an = 0.5,
        kd_pitch_an = 0,
        ki_pitch_an = 0;

double  kp_yaw = 2,
        kd_yaw = 0,
        ki_yaw = 12,
        kp_yaw_an = 1,
        kd_yaw_an = 0,
        ki_yaw_an = 0;

double ang_roll = 0,  ang_pitch = 0, ang_yaw = 0; // PID output outter loop
double u_roll = 0, u_pitch = 0, u_yaw = 0;; // PID output inner loop

PID PID_ax(&anglex, &ang_roll, &ref_roll, kp_roll_an, ki_roll_an, kd_roll_an, DIRECT);
PID PID_gx(&gyrox, &u_roll, &ang_roll, kp_roll, ki_roll, kd_roll, DIRECT);
PID PID_ay(&angley, &ang_pitch, &ref_pitch, kp_pitch_an, ki_pitch_an, kd_pitch_an, DIRECT);
PID PID_gy(&gyroy, &u_pitch, &ang_pitch, kp_pitch, ki_pitch, kd_pitch, DIRECT);
PID PID_az(&anglez, &ang_yaw, &ref_yaw, kp_yaw_an, ki_yaw_an, kd_yaw_an, DIRECT);
PID PID_gz(&gyroz, &u_yaw, &ang_yaw, kp_yaw, ki_yaw, kd_yaw, DIRECT);

void Init_PID(){
    FLESC.attach(flEscPin); //Generate PWM 
    FRESC.attach(frEscPin);
    RLESC.attach(rlEscPin);
    RRESC.attach(rrEscPin);
    PID_ax.SetMode(AUTOMATIC);
    PID_ax.SetOutputLimits(-127, 127);
    PID_ax.SetSampleTime(10);
    PID_gx.SetMode(AUTOMATIC);
    PID_gx.SetOutputLimits(-127, 127);
    PID_gx.SetSampleTime(10);
    PID_ay.SetMode(AUTOMATIC);
    PID_ay.SetOutputLimits(-127, 127);
    PID_ay.SetSampleTime(10);
    PID_gy.SetMode(AUTOMATIC);
    PID_gy.SetOutputLimits(-127, 127);
    PID_gy.SetSampleTime(10);
    PID_gz.SetMode(AUTOMATIC);
    PID_gz.SetOutputLimits(-127, 127);
    PID_gz.SetSampleTime(10);
}
void Compute_PID(){
    //Roll Calculation
    PID_ax.SetTunings(kp_roll_an, ki_roll_an, kd_roll_an);
    PID_ax.Compute();
    PID_gx.SetTunings(kp_roll, ki_roll, kd_roll);
    PID_gx.Compute();
    //Pitch Calculation
    PID_ay.SetTunings(kp_pitch_an, ki_pitch_an, kd_pitch_an);
    PID_ay.Compute();
    PID_gy.SetTunings(kp_pitch, ki_pitch, kd_pitch);
    PID_gy.Compute();
    // Yaw calculation
    PID_gz.SetTunings(kp_yaw_an, ki_yaw_an, kd_yaw_an);
    PID_gz.Compute();
    PID_gz.SetTunings(kp_yaw, ki_yaw, kd_yaw);
    PID_gz.Compute();
}

void updateMotor(){
    double u_throttle = ref_throttle;
    Compute_PID();
    fl = u_throttle + u_roll + u_pitch - u_yaw;
    fr = u_throttle - u_roll + u_pitch + u_yaw;
    rl = u_throttle + u_roll - u_pitch + u_yaw;
    rr = u_throttle - u_roll - u_pitch - u_yaw;
    //write
    FLESC.writeMicroseconds(fl);
    FRESC.writeMicroseconds(fr);
    RLESC.writeMicroseconds(rl);
    RRESC.writeMicroseconds(rr);
    if ((fl >= 2000) || (fr >= 2000) || (rl >= 2000) || (rr >= 2000)) {
      fl = 2000;
      fr = 2000;
      rl = 2000;
      rr = 2000;
    }

    if ((fl <= 1000) || (fr <= 1000) || (rl <= 1000) || (rr <= 1000)) {
      fl = 1000;
      fr = 1000;
      rl = 1000;
      rr = 1000;
    }
}
/*
fr = - - -
rr = - + +
rl = + + -
fl = + - +
*/