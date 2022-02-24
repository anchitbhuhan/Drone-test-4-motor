#include <math.h>
#include <Arduino.h>



#include "PID.h"
#include "MPU_6050.h"
#include "MotorController.h"

PID::PID()
{
  kp = 0;
  ki = 0;
  kd = 0;
  pid_p = 0;
  pid_i = 0;
  pid_d = 0;
  PID = 0;
  error = 0;
  previous_error = 0;
  desired_angle = 0;
  isActivated = false;
}

void PID::init(String iname, float desired_angle, float ikp = 1, float iki = 0, float ikd = 0)
{
  m_name = iname;
  desired_angle = desired_angle;
  this->kp = ikp;
  this->ki =  iki;
  this->kd = ikd;
  m_timePrev = 0.f;
  m_time = millis();
}

float PID::getcommandvalue(float measurement)
{
  if (!isActivated)
  {
    Serial.print(m_name);
    Serial.println(" Not Activated");
    return 0;
  }
  else
  {
    m_timePrev = m_time;  // the previous time is stored before the actual time read
    m_time = millis();  // actual time read
    float elapsedTime = (m_time - m_timePrev) / 1000;

    /*First calculate the error between the desired angle and
      the real measured angle*/
    error = measurement - desired_angle;

    /*Next the proportional value of the PID is just a proportional constant
      multiplied by the error*/

    pid_p = kp * error;

    /*The integral part should only act if we are close to the
      desired position but we want to fine tune the error. That's
      why I've made a if operation for an error between -2 and 2 degree.
      To integrate we just sum the previous integral value with the
      error multiplied by  the integral constant. This will integrate (increase)
      the value each loop till we reach the 0 point*/
    if (-3 < error < 3)
    {
      pid_i = pid_i + (ki * error);
    }

    /*The last part is the derivate. The derivate acts upon the speed of the error.
      As we know the speed is the amount of error that produced in a certain amount of
      time divided by that time. For taht we will use a variable called previous_error.
      We substract that value from the actual error and divide all by the elapsed time.
      Finnaly we multiply the result by the derivate constant*/

    pid_d = kd * ((error - previous_error) / elapsedTime);

    Serial.print(m_name); Serial.print("\t");
    Serial.print(kp); Serial.print(" "); Serial.print(ki); Serial.print(" "); Serial.print(kd); Serial.print("\t");
    Serial.print(error);
    Serial.print("\tp: "); Serial.print(pid_p);
    Serial.print("\ti: "); Serial.print(pid_i);
    Serial.print("\td: "); Serial.println(pid_d);

    /*The final PID values is the sum of each of this 3 parts*/
    PID = pid_p + pid_i + pid_d;

    /*We know taht the min value of PWM signal is 1000us and the max is 2000. So that
      tells us that the PID value can/s oscilate more than -1000 and 1000 because when we
      have a value of 2000us the maximum value taht we could sybstract is 1000 and when
      we have a value of 1000us for the PWM sihnal, the maximum value that we could add is 1000
      to reach the maximum 2000us*/
    if (PID < -MAX_CONTROLLER_OUTPUT)
    {
      PID = -MAX_CONTROLLER_OUTPUT;
    }
    if (PID > MAX_CONTROLLER_OUTPUT)
    {
      PID = MAX_CONTROLLER_OUTPUT;
    }

    previous_error = error; //Remember to store the previous error.
    return PID;
  }

}
