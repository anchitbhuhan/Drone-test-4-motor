#ifndef MotorController_h
#define MotorController_h

#include <Arduino.h>
#include <Servo.h>
#include "MPU_6050.h"

#define MIN_PWM 1000
#define MAX_PWM  2000


class MotorController
{

    //===================================Methods================================//
  public:
    MotorController();
    void init(void);
    void run(void);


    //===================================Fields================================//
  public:
    Servo M1;
    Servo M2;
    Servo M3;
    Servo M4;

    int escPin1  = A0;
    int escPin2  = A1;
    int escPin3  = A2;
    int escPin4  = A3;
    int counter = 0;
    int motorSpeed1 = 0;
    int motorSpeed2 = 0;
    int motorSpeed3 = 0;
    int motorSpeed4 = 0;
};

#endif;
