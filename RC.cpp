#include <Wire.h>
#include <math.h>
#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "RC.h"
#include "FlightController.h"

const byte address[6] = "00001";


void RC::init()
{
  //Create Radio object
  m_rad = new RF24(7, 8);

  //Radio Begin
  m_rad->begin();

  //Open reading pipe
  m_rad->openReadingPipe(0, address);

  //Auto Acknowledgement
  m_rad->setAutoAck(false);

  //Set Data Rate
  m_rad->setDataRate(RF24_2MBPS);

  //Set PA Level
  m_rad->setPALevel(RF24_PA_MIN);

  //Set to Receiving Mode
  m_rad->startListening();

  //Reset Data to Default
  resetData();
  
  //  Serial.println("RC Init");

}

bool RC::isAvailable(FlightController* fc)
{

  bool isradioAvailable = m_rad->available();

  if (isradioAvailable) {
    m_rad->read(&data, sizeof(Data_Package));
    lastReceiveTime = millis();

  // Serial.println("Radio available");
  }
  else {
  // Serial.println("Radio unavailable!");
  }

  currentTime = millis();
  if ( isradioAvailable == false) {// || currentTime - lastReceiveTime > 1000 ) {
    resetData();
  }

  Command* command = &fc->m_command;
  RC_data_to_command(&data, command);
  return isradioAvailable;
}

void RC::RC_data_to_command (Data_Package* dp, Command* command)
{
  
  // Mapping RC values to Motor Outputs
  command->thrust =map(dp->j1PotX, 0, 255, 0, MOTOR_MAX );
  command->yaw = map(dp->j1PotY, 0, 255, 0, MOTOR_MAX );
  command->pitch = map(dp->j2PotX, 0, 255, 0, MOTOR_MAX);
  command->roll = map(dp->j2PotY, 0, 255, 0, MOTOR_MAX);
  command->isArmed = dp->tSwitch1;
}

void RC::resetData() {

  // Reset the values when there is no radio connection - Set initial default values
  data.j1PotX = 127;
  data.j1PotY = 127;
  data.j2PotX = 127;
  data.j2PotY = 127;
  data.tSwitch1 = 0;
}

void RC::print(Data_Package *dp)
{
  //  Serial.print(dp->j1PotX);
  //  Serial.print(" ");
  //  Serial.print(dp->j1PotY);
  //  Serial.print(" ");
  //  Serial.print(dp->j2PotX);
  //  Serial.print(" ");
  //  Serial.print(dp->j2PotY);
  //  Serial.print(" ");
  //  Serial.println(dp->tSwitch1);
}

void RC::printCommand(Command *cmd)
{

  if (cmd != NULL)
  {
    //    Serial.print(cmd->thrust);
    //    Serial.print(" ");
    //    Serial.print(cmd->roll);
    //    Serial.print(" ");
    //    Serial.print(cmd->pitch);
    //    Serial.print(" ");
    //    Serial.println(cmd->yaw);
  }
  else
  {
    //    Serial.println("Command is NULL");
  }
}
RC::~RC()
{
  delete m_rad;
}
