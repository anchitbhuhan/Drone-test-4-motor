#ifndef MotorController_h
#define MotorController_h

#include <Arduino.h>
#include <Servo.h>
#include "MPU_6050.h"

#define MIN_PWM 1000
#define MAX_PWM  2000

#define MOTOR_MAX 100
#define MAX_CONTROLLER_OUTPUT 20



class MotorController
{

    //===================================Methods================================//
  public:
    MotorController();
    void init(void);
    void run(void);
    void printSpeeds();
    void printSpeedsln();

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

    float motorSpeed1 = 0;
    float motorSpeed2 = 0;
    float motorSpeed3 = 0;
    float motorSpeed4 = 0;

    //    float motor1Offset = 5;
    //    float motor2Offset = 0;
    //    float motor3Offset = 5;
    //    float motor4Offset = 0;
    float motor1Offset = 8;
    float motor2Offset = -10;
    float motor3Offset = 0;
    float motor4Offset = 0;

  private:
    bool runMotor1;
    bool runMotor2;
    bool runMotor3;
    bool runMotor4;
};

#endif;
