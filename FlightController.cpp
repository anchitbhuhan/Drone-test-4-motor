#include <Wire.h>
#include <math.h>
#include <Arduino.h>

#include "FlightController.h"
#include "MotorController.h"
#include "MPU_6050.h"

#define INT_MAX 32767

FlightController::FlightController()
{
  Serial.println("FlightController Created");
  this->m_mc = NULL;
  this->m_imu = NULL;
}

void FlightController::init(MPU6050 *imu, MotorController *mc)
{
  this->m_mc  = mc;
  this->m_imu = imu;
}



void FlightController::process()
{
  if (Serial.available()) {
    char temp = Serial.read();
    int val = this->m_mc->motorSpeed1;
    if (temp == 'u') {
      val += 5;
    }
    else if (temp == 'd') {
      val -= 5;
    }

    val = constrain(val, 0, 180);

    this->m_mc->motorSpeed1 = val;
    this->m_mc->motorSpeed2 = val;
    this->m_mc->motorSpeed3 = val;
    this->m_mc->motorSpeed4 = val;

    this->m_mc->run();
  }

  Serial.print("Current speed: "); Serial.println(m_mc->motorSpeed1);
}
