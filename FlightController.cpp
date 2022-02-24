#include <Wire.h>
#include <math.h>
#include <Arduino.h>

#include "utils.h"
#include "FlightController.h"
#include "MotorController.h"
#include "MPU_6050.h"
#include "RC.h"
//#include "PID.h"

#define INT_MAX 32767

FlightController::FlightController() : m_pidroll(), m_pidpitch()
{
  //  Serial.println("FlightController Created");
  this->m_mc = NULL;
  this->m_imu = NULL;
}

void FlightController::init(MPU6050 *imu, MotorController *mc, RC *rad)
{
  //  Serial.println("FC Init");
  this->m_mc  = mc;
  this->m_imu = imu;
  this->m_command.yaw = 0;
  this->m_command.pitch = 0;
  this->m_rad = rad;
  this->m_command.thrust = 0;
  this->m_command.roll = 0;
  //  printCommand();
  //  Serial.println("FC Init Done");
  //  this->m_pid = pid;

//  m_pidroll.init("Roll Controller", 0.f, 0.005, 0.005, 0.01);
//  m_pidpitch.init("Pitch Controller", 0.f, 0.005, 0.005, 0.01);

  m_pidroll.init("Roll Controller", 0.f, 0.3, 0.0, 0.155);
  m_pidpitch.init("Pitch Controller", 0.f, 0.3, 0.0, 0.155);
  m_pidroll.isActivated = false;//true;
  m_pidpitch.isActivated = false;//true;
}



//void FlightController::process()
//{
//  if (Serial.available()) {
//    char temp = Serial.read();
//    int val = this->m_mc->motorSpeed1;
//    if (temp == 'u') {
//      val += 5;
//    }
//    else if (temp == 'd') {
//      val -= 5;
//    }
//
//    val = constrain(val, 0, 180);
//
//    this->m_mc->motorSpeed1 = val;
//    this->m_mc->motorSpeed2 = val;
//    this->m_mc->motorSpeed3 = val;
//    this->m_mc->motorSpeed4 = val;
//
//    this->m_mc->run();
//  }
//
//  Serial.print("Current speed: "); Serial.println(m_mc->motorSpeed1);
//}

//void FlightController::process()
//{
//  int val3 = this->m_mc->motorSpeed3;
//  int val4 = this->m_mc->motorSpeed4;
//
//  if (Serial.available()) {
//    char temp = Serial.read();
//    if (temp == 'l') {
//      val3 = Serial.parseInt();
//      //      char temp1 = Serial.read();
//      //      if (temp1 == 'i')
//      //      {
//      //        val3 += 5;
//      //      }
//      //      else
//      //      {
//      //        val3 -= 5;
//      //      }
//
//    }
//    else if (temp == 'r') {
//      val4 = Serial.parseInt();
//      //      char temp2 = Serial.read();
//      //      if (temp2 == 'i')
//      //      {
//      //        val4 += 5;
//      //      }
//      //      else
//      //      {
//      //        val4 -= 5;
//      //      }
//    }
//
//    val3 = constrain(val3, 0, 180);
//    val4 = constrain(val4, 0, 180);
//
//    this->m_mc->motorSpeed3 = val3;
//    this->m_mc->motorSpeed4 = val4;
//
//    this->m_mc->run();
//  }
//
//  Serial.print("Current speed: Left = "); Serial.println(m_mc->motorSpeed3);
//  Serial.print("Current speed: Right = "); Serial.println(m_mc->motorSpeed4);
//}

//void FlightController::process()
//{
//  int val3 = this->m_mc->motorSpeed3;
//  int val4 = this->m_mc->motorSpeed4;
//
//  String str = this->m_rad->data.s;
//
//  char *token;
//  char str_array[str.length() + 1];
//  str.toCharArray(str_array, str.length());
//  token = strtok(str_array, " ");
//
//  if (token == 'l') {
//    String ss  = strtok(NULL, " ");
//    val3 = ss.toInt();
//
//  }
//  else if (token == 'r') {
//    String ss1  = strtok(NULL, " ");
//    val4 = ss1.toInt();
//  }
//
//  val3 = constrain(val3, 0, 180);
//  val4 = constrain(val4, 0, 180);
//
//  this->m_mc->motorSpeed3 = val3;
//  this->m_mc->motorSpeed4 = val4;
//
//  this->m_mc->run();
//
//
//  Serial.print("Current speed: Left = "); Serial.println(m_mc->motorSpeed3);
//  Serial.print("Current speed: Right = "); Serial.println(m_mc->motorSpeed4);
//}

void FlightController::CommandToSpeed()
{
  int factor = 1;

  if (!m_command.isOutdoor)
  {
    factor = 0.f;
  }

  float rollcommand    = m_pidroll.getcommandvalue(m_imu->roll);
  float pitchcommand = m_pidpitch.getcommandvalue(m_imu->pitch);
  Serial.print("Thrust: "); Serial.print(m_command.thrust);
  Serial.print("\tRoll Command: "); Serial.print(rollcommand);
  Serial.print("\tPitch command: "); Serial.println(pitchcommand);
//  Utils::debug_print(Token_Controller, F("%f"), pitchcommand);
  this->m_mc->motorSpeed1 = constrain((m_command.thrust  +  rollcommand +  pitchcommand + m_mc->motor1Offset) * factor, 0, MOTOR_MAX);
  this->m_mc->motorSpeed2 = constrain((m_command.thrust  - rollcommand - pitchcommand + m_mc->motor2Offset) * factor, 0, MOTOR_MAX);
  this->m_mc->motorSpeed3 = constrain((m_command.thrust  +  rollcommand - pitchcommand + m_mc->motor3Offset) * factor, 0, MOTOR_MAX);
  this->m_mc->motorSpeed4 = constrain((m_command.thrust  - rollcommand +  pitchcommand + m_mc->motor4Offset) * factor, 0, MOTOR_MAX);
}

void FlightController::process()
{
  if ( this->m_rad->isAvailable(this)) {
    CommandToSpeed();
    m_mc->printSpeedsln();
    m_mc->run();
  }


  //  if (this->m_rad->isAvailable(this))
  //  {
  //    CommandToSpeed();
  //    m_mc->printSpeedsln();
  //    m_mc->run();
  //  }
  //  else
  //  {
  //    //    Serial.println("Radio UnAvailable");
  //  }

}


void FlightController::printCommand()
{
  //  Serial.print("Command:  ");
  //  Serial.print(this->m_command.thrust); Serial.print("\t");
  //  Serial.print(this->m_command.roll); Serial.print("\t");
  //  Serial.print(this->m_command.pitch); Serial.print("\t");
  //  Serial.print(this->m_command.isOutdoor); Serial.print("\t");
  //  Serial.println(this->m_command.yaw);
}

void FlightController::printCommandln()
{
  printCommand();
  //    Serial.println();
}
