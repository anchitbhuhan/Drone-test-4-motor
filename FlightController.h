#ifndef Flightcontroller_h
#define Flightcontroller__h


#include "MPU_6050.h"
#include "MotorController.h"


class FlightController
{
  public:
    FlightController();
    void init(MPU6050 * imu, MotorController *mc);
    void process();

  public:
    MotorController *m_mc = NULL;
    MPU6050            *m_imu = NULL;

};

#endif;
