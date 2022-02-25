#ifndef Flightcontroller_h
#define Flightcontroller__h

#include "MPU_6050.h"
#include "MotorController.h"
#include "RC.h"
#include "PID.h"

struct Command
{
  byte thrust;
  byte roll;
  byte pitch;
  byte yaw;
  bool isArmed;
};

class FlightController
{
public:
  FlightController();
  void init(MPU6050 *imu, MotorController *mc, RC *rad);
  void process();
  void RC_data_to_command(Data_Package *dp, Command *command);
  void printCommand();
  void printCommandln();
  void CommandToSpeed();

public:
  MotorController *m_mc = NULL;
  MPU6050 *m_imu = NULL;
  RC *m_rad = NULL;

  Command m_command;

  Data_Package m_data_package;
  
  PID m_pidroll;
  PID m_pidpitch;
};

#endif;
