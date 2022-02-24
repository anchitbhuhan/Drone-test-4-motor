#ifndef RC_h
#define RC_h



#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
//#include "FlightController.h"
class FlightController;

struct Data_Package {
  byte j1PotX;
  byte j1PotY;
  byte j2PotX;
  byte j2PotY;
  byte tSwitch1;
};

//Data_Package data; //Create a variable with the above structure

struct Command ;

class RC
{
  public:

    
    void init();
    bool isAvailable(FlightController * fc);
    void resetData();
    void print(Data_Package *dp);
    void RC_data_to_command (Data_Package* dp, Command* command);
    void printCommand(Command *cmd);
   
    
    int                        waitT                   = 250;
    unsigned long    lastReceiveTime = 0;
    unsigned long    currentTime       = 0;
    Data_Package     data;
    RF24                   *m_rad                 = NULL;
    ~RC();
};


#endif;
