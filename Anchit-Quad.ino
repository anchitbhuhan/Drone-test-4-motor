#define SERIAL_PORT_SPEED 115200

#include "MPU_6050.h"
#include "FlightController.h"
#include "MotorController.h"
#include "PID.h"
#include "RC.h"
#include "utils.h"

MPU6050            imu;
MotorController motor_controller;
FlightController  flight_controller;
RC rc;
unsigned long startTime = 0;
unsigned long currTime = 0;

unsigned long controller_start_time = 15000; // 15 seconds

//#define WINDOW_SIZE 1
//
//int INDEX = 0;
//int VALUE = 0;
//int SUMRoll = 0;
//int READINGS[WINDOW_SIZE];
//int AVERAGEDRoll = 0;
//
//int INDEX2 = 0;
//int VALUE2 = 0;
//int SUMPitch = 0;
//int READINGS2[WINDOW_SIZE];
//int AVERAGEDPitch = 0;



//int waitT = 250;
//unsigned long lastReceiveTime = 0;
//unsigned long currentTime = 0;

//PID pid;

//Debugger debugger;

void setup() {
  Serial.begin(SERIAL_PORT_SPEED);

  imu.begin();
  motor_controller.init();
  //  pid.init();
  rc.init();
  flight_controller.init(&imu, &motor_controller, &rc);
  startTime = millis();
}

void loop() {
  currTime = millis();
  while (imu.update_sensor_values())
  {
    //    timePrev = time;  // the previous time is stored before the actual time read
    //    time = millis();  // actual time read
    //    elapsedTime = (time - timePrev) / 1000;

//    Utils::debug_print(Token_Sensor, F("%f, %f\n"), imu.roll, imu.pitch);
//        Serial.print(imu.roll);
//        Serial.print(",");
//        Serial.println(imu.pitch);
//        Serial.print(",");
    //    Serial.println(imu.yaw);

//    if (currTime - startTime > controller_start_time)
      flight_controller.process();
  }
  //  delay(100);
}
