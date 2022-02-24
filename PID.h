#ifndef PID_h
#define PID_h

#include <Arduino.h>

class PID
{

   public:
              PID();
              void init(String iname, float desired_angle, float ikp = 1, float iki = 0, float ikd = 0);
              float getcommandvalue(float measurement);


    public:
               float PID, error, previous_error;
                float pid_p=0;
                float pid_i=0;
                float pid_d=0;

                float kp=3.55;//3.55
                float ki=0.005;//0.003
                float kd=2.05;//2.05

                bool isActivated;

                float desired_angle = 0; //This is the angle in which we whant the
                         //balance to stay steady
                float m_timePrev, m_time;
                String m_name;
};

#endif
