/*
  MPU6050.h - Header file for the MPU6050 Triple Axis Gyroscope & Accelerometer Arduino Library.

  Version: 1.0.3
  (c) 2014-2015 Anchit Bhushan
*/

#include <Wire.h>
#include <math.h>
#include <Arduino.h>
#include "MPU_6050.h"

float movingAverage(float value)
{

  const byte nvalues = 8;  // Moving average window size
  static byte current = 0; // Index for current value
  static byte cvalues = 0; // Count of values read (<= nvalues)
  static float sum = 0;    // Rolling sum
  static float values[nvalues];

  sum += value;

  // If the window is full, adjust the sum by deleting the oldest value
  if (cvalues == nvalues)
    sum -= values[current];

  values[current] = value; // Replace the oldest with the latest

  if (++current >= nvalues)
    current = 0;

  if (cvalues < nvalues)
    cvalues += 1;

  return sum / cvalues;
}

MPU6050::MPU6050() : mov_roll(10), mov_pitch(10)
{
}

bool MPU6050::begin(mpu6050_dps_t scale = MPU6050_SCALE_2000DPS, mpu6050_range_t range = MPU6050_RANGE_2G, int mpua = MPU6050_ADDRESS)
{
  //  Serial.println("IMU Begin");

  // Initializing MPU
  mpuAddress = mpua;
  Wire.begin();

  // Moving Average for Roll and pitch Values
  mov_roll.begin();
  mov_pitch.begin();

  // Reset calibrate values
  dg.XAxis = 0;
  dg.YAxis = 0;
  dg.ZAxis = 0;
  useCalibrate = false;

  // Reset threshold values
  tg.XAxis = 0;
  tg.YAxis = 0;
  tg.ZAxis = 0;
  actualThreshold = 0;

  // Set Clock Source
  setClockSource(MPU6050_CLOCK_PLL_XGYRO);

  // Set scale and Range for Gyro and Accelero respectively
  setScale(scale);
  setRange(range);

  // Disable Sleep Mode
  setSleepEnabled(false);

  return true;
}

void MPU6050::setRange(mpu6050_range_t range)
{
  uint8_t value;

  // What is the logic of setting rangePerDigit here ?
  switch (range)
  {
  case MPU6050_RANGE_2G:
    rangePerDigit = .000061f;
    break;
  case MPU6050_RANGE_4G:
    rangePerDigit = .000122f;
    break;
  case MPU6050_RANGE_8G:
    rangePerDigit = .000244f;
    break;
  case MPU6050_RANGE_16G:
    rangePerDigit = .0004882f;
    break;
  default:
    break;
  }

  value = readRegister8(MPU6050_REG_ACCEL_CONFIG);
  value &= 0b11100111;
  value |= (range << 3);
  writeRegister8(MPU6050_REG_ACCEL_CONFIG, value);
}

void MPU6050::setScale(mpu6050_dps_t scale)
{
  uint8_t value;

  switch (scale)
  {
  case MPU6050_SCALE_250DPS:
    dpsPerDigit = .007633f;
    break;
  case MPU6050_SCALE_500DPS:
    dpsPerDigit = .015267f;
    break;
  case MPU6050_SCALE_1000DPS:
    dpsPerDigit = .030487f;
    break;
  case MPU6050_SCALE_2000DPS:
    dpsPerDigit = .060975f;
    break;
  default:
    break;
  }

  value = readRegister8(MPU6050_REG_GYRO_CONFIG);
  value &= 0b11100111;
  value |= (scale << 3);
  writeRegister8(MPU6050_REG_GYRO_CONFIG, value);
}

mpu6050_dps_t MPU6050::getScale(void)
{
  uint8_t value;
  value = readRegister8(MPU6050_REG_GYRO_CONFIG);
  value &= 0b00011000;
  value >>= 3;
  return (mpu6050_dps_t)value;
}

mpu6050_range_t MPU6050::getRange(void)
{
  uint8_t value;
  value = readRegister8(MPU6050_REG_ACCEL_CONFIG);
  value &= 0b00011000;
  value >>= 3;
  return (mpu6050_range_t)value;
}

float MPU6050::readTemperature(void)
{
  int16_t T;
  T = readRegister16(MPU6050_REG_TEMP_OUT_H);
  return (float)T / 340 + 36.53;
}

void MPU6050::setClockSource(mpu6050_clockSource_t source)
{
  uint8_t value;
  value = readRegister8(MPU6050_REG_PWR_MGMT_1);
  value = value & 0b11111000;
  value = value | source;
  writeRegister8(MPU6050_REG_PWR_MGMT_1, value);
}

//=====================Update Sensor Values=============================//

bool MPU6050::update_sensor_values(void)
{
  bool updated = false;

  if ((millis() - accel_update_timer) > 20)  // ~50 hz
  { 
    update_accel();
    accel_update_timer = millis();
    updated = true;
  }

  if ((micros() - gyro_update_timer) > 1300)  // ~800 Hz
  { 
    update_gyro();
    gyro_update_timer = micros();
    updated = true;
  }

  if (updated)
  {
    combine();
  }

  return updated;
}

void MPU6050::update_accel()
{

  Vector C;
  Vector scaled_acc = readScaledAccel();

  // Vector (H-b)
  Vector h_b;
  h_b.XAxis = scaled_acc.XAxis - B[0];
  h_b.YAxis = scaled_acc.YAxis - B[1];
  h_b.ZAxis = scaled_acc.ZAxis - B[2];

  // A' = A* (H-b)
  C.XAxis = A[0][0] * h_b.XAxis + A[0][1] * h_b.YAxis + A[0][2] * h_b.ZAxis;
  C.YAxis = A[1][0] * h_b.XAxis + A[1][1] * h_b.YAxis + A[1][2] * h_b.ZAxis;
  C.ZAxis = A[2][0] * h_b.XAxis + A[2][1] * h_b.YAxis + A[2][2] * h_b.ZAxis;

  // Ay = Taninv(-Ax/(Ay**2 + Az**2))
  acc_pitch = atan2(-C.XAxis, sqrt(C.YAxis * C.YAxis + C.ZAxis * C.ZAxis)) * 180.0 / M_PI;

  // Ax = Taninv(Ay / Az)
  acc_roll = (atan2(C.YAxis, C.ZAxis) * 180.0) / M_PI;
}

void MPU6050::update_gyro()
{

  timer = millis();
  Vector norm_gyro = readNormalizeGyro();

  gyro_pitch = pitch + norm_gyro.YAxis * timeStep;
  gyro_roll = roll + norm_gyro.XAxis * timeStep;
  gyro_yaw = yaw + norm_gyro.ZAxis * timeStep;
}

void MPU6050::combine()
{

  // float dt = (float)(micros() - combination_update_timer);

  // Angle calculation through Complementary filter
  roll = GYRO_PART * gyro_roll + ACC_PART * acc_roll;
  pitch = GYRO_PART * gyro_pitch + ACC_PART * acc_pitch;
  yaw = gyro_yaw;

  // Calculating Moving Average
  roll = movingAverage(roll);
  pitch = movingAverage(pitch);

  // combination_update_timer = micros();
  delay((timeStep * 1000) - (millis() - timer));
}

//=======================Get Accelaration and Gyro Offsets==========================//
int16_t MPU6050::getAccelOffsetX(void)
{
  return readRegister16(MPU6050_REG_ACCEL_XOFFS_H);
}

int16_t MPU6050::getAccelOffsetY(void)
{
  return readRegister16(MPU6050_REG_ACCEL_YOFFS_H);
}

int16_t MPU6050::getAccelOffsetZ(void)
{
  return readRegister16(MPU6050_REG_ACCEL_ZOFFS_H);
}

int16_t MPU6050::getGyroOffsetX(void)
{
  return readRegister16(MPU6050_REG_GYRO_XOFFS_H);
}

int16_t MPU6050::getGyroOffsetY(void)
{
  return readRegister16(MPU6050_REG_GYRO_YOFFS_H);
}

int16_t MPU6050::getGyroOffsetZ(void)
{
  return readRegister16(MPU6050_REG_GYRO_ZOFFS_H);
}

//=======================Set Accelaration and Gyro Offsets==========================//

void MPU6050::setAccelOffsetX(int16_t offset)
{
  writeRegister16(MPU6050_REG_ACCEL_XOFFS_H, offset);
}

void MPU6050::setAccelOffsetY(int16_t offset)
{
  writeRegister16(MPU6050_REG_ACCEL_YOFFS_H, offset);
}

void MPU6050::setAccelOffsetZ(int16_t offset)
{
  writeRegister16(MPU6050_REG_ACCEL_ZOFFS_H, offset);
}

void MPU6050::setGyroOffsetX(int16_t offset)
{
  writeRegister16(MPU6050_REG_GYRO_XOFFS_H, offset);
}

void MPU6050::setGyroOffsetY(int16_t offset)
{
  writeRegister16(MPU6050_REG_GYRO_YOFFS_H, offset);
}

void MPU6050::setGyroOffsetZ(int16_t offset)
{
  writeRegister16(MPU6050_REG_GYRO_ZOFFS_H, offset);
}

//=====================Sleep========================================//
void MPU6050::setSleepEnabled(bool state)
{
  writeRegisterBit(MPU6050_REG_PWR_MGMT_1, 6, state);
}

//=====================Reading Accelaration========================================//

Vector MPU6050::readRawAccel(void)
{
  Wire.beginTransmission(mpuAddress);
  Wire.write(MPU6050_REG_ACCEL_XOUT_H);
  Wire.endTransmission();

  Wire.beginTransmission(mpuAddress);
  Wire.requestFrom(mpuAddress, 6);

  uint8_t xha = Wire.read();
  uint8_t xla = Wire.read();
  uint8_t yha = Wire.read();
  uint8_t yla = Wire.read();
  uint8_t zha = Wire.read();
  uint8_t zla = Wire.read();

  ra.XAxis = xha << 8 | xla;
  ra.YAxis = yha << 8 | yla;
  ra.ZAxis = zha << 8 | zla;

  return ra;
}

Vector MPU6050::readNormalizeAccel(void)
{
  readRawAccel();

  na.XAxis = ra.XAxis * rangePerDigit * 9.80665f;
  na.YAxis = ra.YAxis * rangePerDigit * 9.80665f;
  na.ZAxis = ra.ZAxis * rangePerDigit * 9.80665f;

  return na;
}

Vector MPU6050::readScaledAccel(void)
{
  readRawAccel();

  na.XAxis = ra.XAxis * rangePerDigit;
  na.YAxis = ra.YAxis * rangePerDigit;
  na.ZAxis = ra.ZAxis * rangePerDigit;

  return na;
}

//============================Set Thershold and Get Threshold for Gyro==================//
// Calibrate algorithm
void MPU6050::calibrateGyro(uint8_t samples)
{
  // Set calibrate
  useCalibrate = true;

  // Reset values
  float sumX = 0;
  float sumY = 0;
  float sumZ = 0;
  float sigmaX = 0;
  float sigmaY = 0;
  float sigmaZ = 0;

  // Read n-samples
  for (uint8_t i = 0; i < samples; ++i)
  {
    readRawGyro();
    sumX += rg.XAxis;
    sumY += rg.YAxis;
    sumZ += rg.ZAxis;

    sigmaX += rg.XAxis * rg.XAxis;
    sigmaY += rg.YAxis * rg.YAxis;
    sigmaZ += rg.ZAxis * rg.ZAxis;

    delay(5);
  }

  // Calculate delta vectors
  dg.XAxis = sumX / samples;
  dg.YAxis = sumY / samples;
  dg.ZAxis = sumZ / samples;

  // Calculate threshold vectors
  th.XAxis = sqrt((sigmaX / 50) - (dg.XAxis * dg.XAxis));
  th.YAxis = sqrt((sigmaY / 50) - (dg.YAxis * dg.YAxis));
  th.ZAxis = sqrt((sigmaZ / 50) - (dg.ZAxis * dg.ZAxis));

  // If already set threshold, recalculate threshold vectors
  if (actualThreshold > 0)
  {
    setThreshold(actualThreshold);
  }
}

// Get current threshold value
uint8_t MPU6050::getThreshold(void)
{
  return actualThreshold;
}

// Set treshold value
void MPU6050::setThreshold(uint8_t multiple)
{
  if (multiple > 0)
  {
    // If not calibrated, need calibrate
    if (!useCalibrate)
    {
      calibrateGyro();
    }

    // Calculate threshold vectors
    tg.XAxis = th.XAxis * multiple;
    tg.YAxis = th.YAxis * multiple;
    tg.ZAxis = th.ZAxis * multiple;
  }
  else
  {
    // No threshold
    tg.XAxis = 0;
    tg.YAxis = 0;
    tg.ZAxis = 0;
  }

  // Remember old threshold value
  actualThreshold = multiple;
}

//=====================Reading Gyro========================================//
Vector MPU6050::readRawGyro(void)
{
  Wire.beginTransmission(mpuAddress);
  Wire.write(MPU6050_REG_GYRO_XOUT_H);
  Wire.endTransmission();

  Wire.beginTransmission(mpuAddress);
  Wire.requestFrom(mpuAddress, 6);

  while (Wire.available() < 6)
    ;

  uint8_t xha = Wire.read();
  uint8_t xla = Wire.read();
  uint8_t yha = Wire.read();
  uint8_t yla = Wire.read();
  uint8_t zha = Wire.read();
  uint8_t zla = Wire.read();

  rg.XAxis = xha << 8 | xla;
  rg.YAxis = yha << 8 | yla;
  rg.ZAxis = zha << 8 | zla;

  return rg;
}

Vector MPU6050::readNormalizeGyro(void)
{
  readRawGyro();

  if (useCalibrate)
  {
    ng.XAxis = (rg.XAxis - dg.XAxis) * dpsPerDigit;
    ng.YAxis = (rg.YAxis - dg.YAxis) * dpsPerDigit;
    ng.ZAxis = (rg.ZAxis - dg.ZAxis) * dpsPerDigit;
  }
  else
  {
    ng.XAxis = rg.XAxis * dpsPerDigit;
    ng.YAxis = rg.YAxis * dpsPerDigit;
    ng.ZAxis = rg.ZAxis * dpsPerDigit;
  }

  if (actualThreshold)
  {
    if (abs(ng.XAxis) < tg.XAxis)
      ng.XAxis = 0;
    if (abs(ng.YAxis) < tg.YAxis)
      ng.YAxis = 0;
    if (abs(ng.ZAxis) < tg.ZAxis)
      ng.ZAxis = 0;
  }

  return ng;
}

//===============================Calibrate Gyro and Accelerometer==========================//

void MPU6050::setAccelOffset()
{

  // reset offsets
  setAccelOffsetX(0);
  setAccelOffsetY(0);
  setAccelOffsetZ(0);

  if (state == 0)
  {
    //    Serial.println("\nReading Accel sensors for first time...");
    meansensorsAcc();
    state++;
    delay(1000);
  }

  if (state == 1)
  {
    //    Serial.println("\nCalculating offsets...");
    calibrationAcc();
    state++;
    delay(1000);
  }

  if (state == 2)
  {
    meansensorsAcc();
    //    Serial.println("\nFINISHED!");
    //    Serial.print("\nSensor readings with offsets:\t");
    //    Serial.print(mean_ax);
    //    Serial.print("\t");
    //    Serial.print(mean_ay);
    //    Serial.print("\t");
    //    Serial.print(mean_az);
    //    Serial.print("\t");
    //    Serial.print("Your offsets:\t");
    //    Serial.print(ax_offset);
    //    Serial.print("\t");
    //    Serial.print(ay_offset);
    //    Serial.print("\t");
    //    Serial.print(az_offset);
    //    Serial.print("\t");
    //    Serial.println("\nData is printed as: acelX acelY acelZ");
    //    Serial.println("Check that your sensor readings are close to 0 0 16384");
    //    Serial.println("If calibration was succesful write down your offsets so you can set them in your projects using something similar to mpu.setXAccelOffset(youroffset)");

    setAccelOffsetX(ax_offset);
    setAccelOffsetY(ay_offset);
    setAccelOffsetZ(az_offset);
    state = 0;
  }
}

void MPU6050::meansensorsAcc()
{
  long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;

  while (i < (buffersize + 101))
  {

    Vector raw_Acc = readRawAccel();
    ax = raw_Acc.XAxis;
    ay = raw_Acc.YAxis;
    az = raw_Acc.ZAxis;

    if (i > 100 && i <= (buffersize + 100))
    { // First 100 measures are discarded
      buff_ax = buff_ax + ax;
      buff_ay = buff_ay + ay;
      buff_az = buff_az + az;
    }
    if (i == (buffersize + 100))
    {
      mean_ax = buff_ax / buffersize;
      mean_ay = buff_ay / buffersize;
      mean_az = buff_az / buffersize;
    }
    i++;
    delay(2); // Needed so we don't get repeated measures
  }
}

void MPU6050::calibrationAcc()
{
  ax_offset = -mean_ax / 8;
  ay_offset = -mean_ay / 8;
  az_offset = (16384 - mean_az) / 8;

  while (1)
  {
    int ready = 0;
    setAccelOffsetX(ax_offset);
    setAccelOffsetY(ay_offset);
    setAccelOffsetZ(az_offset);

    meansensorsAcc();
    //    Serial.println("...");

    if (abs(mean_ax) <= acel_deadzone)
      ready++;
    else
      ax_offset = ax_offset - mean_ax / acel_deadzone;

    if (abs(mean_ay) <= acel_deadzone)
      ready++;
    else
      ay_offset = ay_offset - mean_ay / acel_deadzone;

    if (abs(16384 - mean_az) <= acel_deadzone)
      ready++;
    else
      az_offset = az_offset + (16384 - mean_az) / acel_deadzone;

    if (ready == 3)
      break;
  }
}

//==================Calibrate Gyro========================//

void MPU6050::setGyroOffset()
{

  // reset offsets
  setGyroOffsetX(0);
  setGyroOffsetY(0);
  setGyroOffsetZ(0);

  if (state == 0)
  {
    //    Serial.println("\nReading Gyro sensors for first time...");
    meansensorsGyro();
    state++;
    delay(1000);
  }

  if (state == 1)
  {
    //    Serial.println("\nCalculating offsets...");
    calibrationGyro();
    state++;
    delay(1000);
  }

  if (state == 2)
  {
    meansensorsGyro();
    //    Serial.println("\nFINISHED!");
    //    Serial.print("\nSensor readings with offsets:\t");
    //    Serial.print(mean_gx);
    //    Serial.print("\t");
    //    Serial.print(mean_gy);
    //    Serial.print("\t");
    //    Serial.print(mean_gz);
    //    Serial.print("\t");
    //    Serial.print("Your offsets:\t");
    //    Serial.print(gx_offset);
    //    Serial.print("\t");
    //    Serial.print(gy_offset);
    //    Serial.print("\t");
    //    Serial.print(gz_offset);
    //    Serial.print("\t");
    //    Serial.println("\nData is printed as: GyroX GyroY GyroZ");
    //    Serial.println("Check that your sensor readings are close to 0 0 0");
    //    Serial.println("If calibration was succesful write down your offsets so you can set them in your projects using something similar to mpu.setXGyroOffset(youroffset)");

    setGyroOffsetX(gx_offset);
    setGyroOffsetY(gy_offset);
    setGyroOffsetZ(gz_offset);
  }
}

void MPU6050::meansensorsGyro()
{
  long i = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;

  while (i < (buffersize + 101))
  {

    Vector raw_Gyro = readRawGyro();
    gx = raw_Gyro.XAxis;
    gy = raw_Gyro.YAxis;
    gz = raw_Gyro.ZAxis;

    if (i > 100 && i <= (buffersize + 100))
    { // First 100 measures are discarded
      buff_gx = buff_gx + gx;
      buff_gy = buff_gy + gy;
      buff_gz = buff_gz + gz;
    }
    if (i == (buffersize + 100))
    {
      mean_gx = buff_gx / buffersize;
      mean_gy = buff_gy / buffersize;
      mean_gz = buff_gz / buffersize;
    }
    i++;
    delay(2); // Needed so we don't get repeated measures
  }
}

void MPU6050::calibrationGyro()
{
  gx_offset = -mean_gx / 4;
  gy_offset = -mean_gy / 4;
  gz_offset = -mean_gz / 4;

  while (1)
  {
    int ready = 0;
    setGyroOffsetX(gx_offset);
    setGyroOffsetY(gy_offset);
    setGyroOffsetZ(gz_offset);

    meansensorsGyro();
    //    Serial.println("...");
    //    Serial.print("(abs(mean_gx) = ");
    //    Serial.print(abs(mean_gx));
    //    Serial.print(" (abs(mean_gy) = ");
    //    Serial.print(abs(mean_gx));
    //    Serial.print(" (abs(mean_gz) = ");
    //    Serial.println(abs(mean_gx));
    //    Serial.print(" Ready = ");
    //    Serial.println(ready);
    if (abs(mean_gx) <= giro_deadzone)
      ready++;
    else
      gx_offset = gx_offset - mean_gx / (giro_deadzone + 1);

    if (abs(mean_gy) <= giro_deadzone)
      ready++;
    else
      gy_offset = gy_offset - mean_gy / (giro_deadzone + 1);

    if (abs(mean_gz) <= giro_deadzone)
      ready++;
    else
      gz_offset = gz_offset - mean_gz / (giro_deadzone + 1);

    if (ready == 3)
      break;
  }
}

//===============================Setting and Reading from Registers==========================//

uint8_t MPU6050::readRegister8(uint8_t reg)
{
  uint8_t value;

  Wire.beginTransmission(mpuAddress);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.beginTransmission(mpuAddress);
  Wire.requestFrom(mpuAddress, 1);
  value = Wire.read();
  Wire.endTransmission();

  return value;
}

void MPU6050::writeRegister8(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(mpuAddress);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

int16_t MPU6050::readRegister16(uint8_t reg)
{
  int16_t value;
  Wire.beginTransmission(mpuAddress);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.beginTransmission(mpuAddress);
  Wire.requestFrom(mpuAddress, 2);

  uint8_t vha = Wire.read();
  uint8_t vla = Wire.read();

  Wire.endTransmission();

  value = vha << 8 | vla;

  return value;
}

void MPU6050::writeRegister16(uint8_t reg, int16_t value)
{
  Wire.beginTransmission(mpuAddress);

  Wire.write(reg);
  Wire.write((uint8_t)(value >> 8));
  Wire.write((uint8_t)value);
  Wire.endTransmission();
}

bool MPU6050::readRegisterBit(uint8_t reg, uint8_t pos)
{
  uint8_t value;
  value = readRegister8(reg);
  return ((value >> pos) & 1);
}

void MPU6050::writeRegisterBit(uint8_t reg, uint8_t pos, bool state)
{
  uint8_t value;
  value = readRegister8(reg);

  if (state)
  {
    value |= (1 << pos);
  }
  else
  {
    value &= ~(1 << pos);
  }

  writeRegister8(reg, value);
}
