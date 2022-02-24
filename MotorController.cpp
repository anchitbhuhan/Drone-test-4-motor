#include <Wire.h>
#include <math.h>
#include <Arduino.h>
#include <Servo.h>
#include "MotorController.h"
#include "MPU_6050.h"

MotorController::MotorController()
{
  runMotor1 = true;
  runMotor2 = true;
  runMotor3 = true;
  runMotor4 = true;
  //  Serial.println("MotorController Created");
  this->counter = 0;
  //  this->motorSpeed1 = 30;
  //  this->motorSpeed2 = 30;
  //  this->motorSpeed3 = 30;
  //  this->motorSpeed4 = 30;
}

void MotorController::init()
{
  //  Serial.println("MotorController Init");
  M1.attach(escPin1, MIN_PWM, MAX_PWM); // (pin, min pulse width, max pulse width in microseconds)
  M2.attach(escPin2, MIN_PWM, MAX_PWM) ; // (pin, min pulse width, max pulse width in microseconds)
  M3.attach(escPin3, MIN_PWM, MAX_PWM); // (pin, min pulse width, max pulse width in microseconds)
  M4.attach(escPin4, MIN_PWM, MAX_PWM); // (pin, min pulse width, max pulse width in microseconds)

  M1.write(this->motorSpeed1);   //ESC arm command. ESCs won't start unless input speed is less during initialization.
  M2.write(this->motorSpeed2);   //ESC arm command. ESCs won't start unless input speed is less during initialization.
  M3.write(this->motorSpeed3);   //ESC arm command. ESCs won't start unless input speed is less during initialization.
  M4.write(this->motorSpeed4);   //ESC arm command. ESCs won't start unless input speed is less during initialization.

  M1.writeMicroseconds(MIN_PWM);
  M2.writeMicroseconds(MIN_PWM);
  M3.writeMicroseconds(MIN_PWM);
  M4.writeMicroseconds(MIN_PWM);
}


void MotorController::run(void)
{
  //    Serial.print("Motorspeed in run method = ");
  //    Serial.println(motorSpeed1);
  if (runMotor1)
  {
    M1.write(this->motorSpeed1);
  }

  if (runMotor2)
  {
    M2.write(this->motorSpeed2);
  }

  if (runMotor3)
  {
    M3.write(this->motorSpeed3);
  }

  if (runMotor4)
  {
    M4.write(this->motorSpeed4);
  }
}

void MotorController::printSpeeds()
{
  Serial.print("Motor Speeds:  ");
  Serial.print(motorSpeed1); Serial.print("\t");
  Serial.print(motorSpeed2); Serial.print("\t");
  Serial.print(motorSpeed3); Serial.print("\t");
  Serial.print(motorSpeed4);
}

void MotorController::printSpeedsln()
{
  printSpeeds();
  Serial.println();
}
