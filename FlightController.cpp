#include <Wire.h>
#include <math.h>
#include <Arduino.h>

#include "utils.h"
#include "FlightController.h"
#include "MotorController.h"
#include "MPU_6050.h"
#include "RC.h"

#define INT_MAX 32767

FlightController::FlightController() : m_pidroll(), m_pidpitch()
{
  //  Serial.println("FlightController Created");
  this->m_mc = NULL;
  this->m_imu = NULL;
}

void FlightController::init(MPU6050 *imu, MotorController *mc, RC *rad)
{
  // Takes Reference of imu, motor_controller and radio
  this->m_mc = mc;
  this->m_imu = imu;
  this->m_rad = rad;

  // Set initial orientation commands to 0
  this->m_command.yaw = 0;
  this->m_command.pitch = 0;
  this->m_command.thrust = 0;
  this->m_command.roll = 0;

  //  printCommand();
  //  this->m_pid = pid;

  // Set kp, ki & Kd value for Roll and Pitch
  //   m_pidroll.init("Roll Controller", 0.f, 0.005, 0.005, 0.01);
  //   m_pidpitch.init("Pitch Controller", 0.f, 0.005, 0.005, 0.01);
  m_pidroll.init("Roll Controller", 0.f, 0.3, 0.0, 0.155);
  m_pidpitch.init("Pitch Controller", 0.f, 0.3, 0.0, 0.155);

  // Flag to turn On/Off PID on Roll and Pitch
  m_pidroll.isActivated = false;  // true;
  m_pidpitch.isActivated = false; // true;

  //  Serial.println("FC Init");
}

void FlightController::CommandToSpeed()
{
  //Armed
  int factor = 1;

  // Set Factor to when UnArmed.
  if (!m_command.isArmed)
  {
    factor = 0.f;
  }

  // Get commanded roll value from PID controller
  float rollcommand = m_pidroll.getcommandvalue(m_imu->roll);

  // Get commanded pitch value from PID controller
  float pitchcommand = m_pidpitch.getcommandvalue(m_imu->pitch);

  Serial.print("Thrust: ");
  Serial.print(m_command.thrust);
  Serial.print("\tRoll Command: ");
  Serial.print(rollcommand);
  Serial.print("\tPitch command: ");
  Serial.println(pitchcommand);

  //  Utils::debug_print(Token_Controller, F("%f"), pitchcommand);
  this->m_mc->motorSpeed1 = constrain((m_command.thrust + rollcommand + pitchcommand + m_mc->motor1Offset) * factor, 0, MOTOR_MAX);
  this->m_mc->motorSpeed2 = constrain((m_command.thrust - rollcommand - pitchcommand + m_mc->motor2Offset) * factor, 0, MOTOR_MAX);
  this->m_mc->motorSpeed3 = constrain((m_command.thrust + rollcommand - pitchcommand + m_mc->motor3Offset) * factor, 0, MOTOR_MAX);
  this->m_mc->motorSpeed4 = constrain((m_command.thrust - rollcommand + pitchcommand + m_mc->motor4Offset) * factor, 0, MOTOR_MAX);
}

void FlightController::process()
{
  // Is Radio Available ?
  if (this->m_rad->isAvailable(this))
  {
    // Calculate Speed to be given to motor controller
    CommandToSpeed();

    m_mc->printSpeedsln();

    // Run the motor on calculated speed
    m_mc->run();
  }
}

void FlightController::printCommand()
{
  //  Serial.print("Command:  ");
  //  Serial.print(this->m_command.thrust); Serial.print("\t");
  //  Serial.print(this->m_command.roll); Serial.print("\t");
  //  Serial.print(this->m_command.pitch); Serial.print("\t");
  //  Serial.print(this->m_command.isArmed); Serial.print("\t");
  //  Serial.println(this->m_command.yaw);
}

void FlightController::printCommandln()
{
  printCommand();
  //    Serial.println();
}
