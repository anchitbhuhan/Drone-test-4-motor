#define SERIAL_PORT_SPEED 115200

#include "MPU_6050.h"
#include "FlightController.h"
#include "MotorController.h"
//#include "debugger.h"

MPU6050            imu;
MotorController motor_controller;
FlightController  flight_controller;

//Debugger debugger;

void setup() {
  Serial.begin(SERIAL_PORT_SPEED);

  imu.begin();
  motor_controller.init();
  flight_controller.init(&imu, &motor_controller);
}

void loop() {
  while (!imu.update_sensor_values())
  {
    flight_controller.process();
  }

  delay(100);
}
