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
  //  Serial.println("RC Init");
  m_rad = new RF24(7, 8);
  m_rad->begin();
  m_rad->openReadingPipe(0, address);
  m_rad->setAutoAck(false);
  m_rad->setDataRate(RF24_2MBPS);
  m_rad->setPALevel(RF24_PA_MIN);
  m_rad->startListening(); //  Set the module as receiver
  resetData();

}

bool RC::isAvailable(FlightController* fc)
{
  bool isradioAvailable = m_rad->available();

  if (isradioAvailable) {
    m_rad->read(&data, sizeof(Data_Package));
    lastReceiveTime = millis();
//    Serial.println("Radio available");
  }
  else {
//    Serial.println("Radio unavailable!");
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
  int scale = 1;

  if (dp->tSwitch1 == 1)
  {
    scale = 0.5;
  }

  command->thrust =map(dp->j1PotX, 0, 255, 0, MOTOR_MAX );
  command->yaw = map(dp->j1PotY, 0, 255, 0, MOTOR_MAX );
  command->pitch = map(dp->j2PotX, 0, 255, 0, MOTOR_MAX);
  command->roll = map(dp->j2PotY, 0, 255, 0, MOTOR_MAX);
  command->isOutdoor = dp->tSwitch1;

  //  print(dp);
  //  Serial.println(command->thrust);
  //    printCommand(command);
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
  //  Serial.print("j1PotX: ");
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
