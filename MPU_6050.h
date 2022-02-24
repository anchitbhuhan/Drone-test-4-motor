/*
MPU6050.h - Header file for the MPU6050 Triple Axis Gyroscope & Accelerometer Arduino Library.

Version: 1.0.3
(c) 2014-2015 Anchit Bhushan
*/

#ifndef MPU6050_h
#define MPU6050_h
#include <stdint.h>
#include <movingAvg.h> 

#define GYRO_PART 0.985
#define ACC_PART (1.0 - GYRO_PART)


#define MPU6050_ADDRESS                       (0x68)

#define MPU6050_REG_ACCEL_XOFFS_H     (0x06)
#define MPU6050_REG_ACCEL_XOFFS_L      (0x07)

#define MPU6050_REG_ACCEL_YOFFS_H     (0x08)
#define MPU6050_REG_ACCEL_YOFFS_L      (0x09)

#define MPU6050_REG_ACCEL_ZOFFS_H     (0x0A)
#define MPU6050_REG_ACCEL_ZOFFS_L      (0x0B)


#define MPU6050_REG_GYRO_XOFFS_H      (0x13)
#define MPU6050_REG_GYRO_XOFFS_L       (0x14)

#define MPU6050_REG_GYRO_YOFFS_H      (0x15)
#define MPU6050_REG_GYRO_YOFFS_L       (0x16)

#define MPU6050_REG_GYRO_ZOFFS_H      (0x17)
#define MPU6050_REG_GYRO_ZOFFS_L       (0x18)


#define MPU6050_REG_GYRO_CONFIG       (0x1B) // Gyroscope Configuration
#define MPU6050_REG_ACCEL_CONFIG       (0x1C) // Accelerometer Configuration

#define MPU6050_REG_ACCEL_XOUT_H      (0x3B)
#define MPU6050_REG_ACCEL_XOUT_L       (0x3C)
#define MPU6050_REG_ACCEL_YOUT_H      (0x3D)
#define MPU6050_REG_ACCEL_YOUT_L       (0x3E)
#define MPU6050_REG_ACCEL_ZOUT_H      (0x3F)
#define MPU6050_REG_ACCEL_ZOUT_L       (0x40)


#define MPU6050_REG_GYRO_XOUT_H       (0x43)
#define MPU6050_REG_GYRO_XOUT_L        (0x44)
#define MPU6050_REG_GYRO_YOUT_H       (0x45)
#define MPU6050_REG_GYRO_YOUT_L        (0x46)
#define MPU6050_REG_GYRO_ZOUT_H       (0x47)
#define MPU6050_REG_GYRO_ZOUT_L        (0x48)



#define MPU6050_REG_TEMP_OUT_H         (0x41)
#define MPU6050_REG_TEMP_OUT_L          (0x42)

#define MPU6050_REG_PWR_MGMT_1       (0x6B) // Power Management 1


struct Vector
{
    float XAxis;
    float YAxis;
    float ZAxis;
};



typedef enum
{
    MPU6050_RANGE_16G             = 0b11,
    MPU6050_RANGE_8G              = 0b10,
    MPU6050_RANGE_4G              = 0b01,
    MPU6050_RANGE_2G              = 0b00,
} mpu6050_range_t;


typedef enum
{
    MPU6050_SCALE_2000DPS         = 0b11,
    MPU6050_SCALE_1000DPS         = 0b10,
    MPU6050_SCALE_500DPS          = 0b01,
    MPU6050_SCALE_250DPS          = 0b00
} mpu6050_dps_t;

typedef enum
{
    MPU6050_CLOCK_KEEP_RESET      = 0b111,
    MPU6050_CLOCK_EXTERNAL_19MHZ  = 0b101,
    MPU6050_CLOCK_EXTERNAL_32KHZ  = 0b100,
    MPU6050_CLOCK_PLL_ZGYRO       = 0b011,
    MPU6050_CLOCK_PLL_YGYRO       = 0b010,
    MPU6050_CLOCK_PLL_XGYRO       = 0b001,
    MPU6050_CLOCK_INTERNAL_8MHZ   = 0b000
} mpu6050_clockSource_t;

class MPU6050
{
    public:
                MPU6050();
//=====================Start Up Operations==========================================//
                bool begin(mpu6050_dps_t scale = MPU6050_SCALE_2000DPS, mpu6050_range_t range = MPU6050_RANGE_2G, int mpua = MPU6050_ADDRESS);
                void setClockSource(mpu6050_clockSource_t source);                                                                      

//=====================Update Sensor Values=============================//
                bool update_sensor_values(void);
                void update_accel(void);
                void update_gyro(void);
                void combine(void);
                

//=====================Getting and Setting Scale and Range==========================================//
                void setScale(mpu6050_dps_t scale);
                void setRange(mpu6050_range_t range);

                mpu6050_dps_t getScale(void);
                mpu6050_range_t getRange(void);

//=====================Temperature=============================//
                float readTemperature(void);                                                                                                            

//=====================Find and Set Accelaration and Gyro Offsets=============================//
                void meansensorsAcc();
                void calibrationAcc();
                void setAccelOffset();

                void meansensorsGyro();
                void calibrationGyro();
                void setGyroOffset();
                
//=====================Getting and Setting Accelaration and Gyro Offsets=============================//                
                int16_t getAccelOffsetX(void);                                                                                                                                                                                                                      
                void setAccelOffsetX(int16_t offset);                                                                                                   
                int16_t getAccelOffsetY(void);                                                                                                            
                void setAccelOffsetY(int16_t offset);                                                                                                  
                int16_t getAccelOffsetZ(void);                                                                                                           
                void setAccelOffsetZ(int16_t offset);

                int16_t getGyroOffsetX(void);
                void setGyroOffsetX(int16_t offset);
                int16_t getGyroOffsetY(void);
                void setGyroOffsetY(int16_t offset);
                int16_t getGyroOffsetZ(void);
                void setGyroOffsetZ(int16_t offset);

//=====================Sleep===================================================//
                void setSleepEnabled(bool state);                                                                                                    

//=====================Reading Accelaration and Gyro========================================//                
                Vector readRawAccel(void);                                                                                                                
                Vector readNormalizeAccel(void);                                                                                                    
                Vector readScaledAccel(void);

                Vector readRawGyro(void);
                Vector readNormalizeGyro(void);
//=======================Set Threshold and Get thershold=========================//
                void setThreshold(uint8_t multiple = 1);
                uint8_t getThreshold(void);
                void calibrateGyro(uint8_t samples = 50);


////=====================Calibrate Accelerometer and Gyro=========================//
//                void calibrateAccelero(uint8_t samples = 100);


//=====================Reading and Writing to 8 bit and 16 bit registers=========================//                

                uint8_t readRegister8(uint8_t reg);                                                                                                            
                void writeRegister8(uint8_t reg, uint8_t value);                                                                                         


                int16_t readRegister16(uint8_t reg);                                                                                                         
                void writeRegister16(uint8_t reg, int16_t value);                                                                                        

                bool readRegisterBit(uint8_t reg, uint8_t pos);
                void writeRegisterBit(uint8_t reg, uint8_t pos, bool state);

                public:
                    Vector ra, rg; // Raw vectors
                    Vector na, ng; // Normalized vectors
                    Vector tg, dg; // Threshold and Delta for Gyro
                    Vector th;     // Threshold
                    float rangePerDigit,  dpsPerDigit;
                    float actualThreshold;
                    int mpuAddress;
                    int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
                    int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
                    int giro_deadzone=1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
                    bool useCalibrate;
                    int16_t ax, ay, az,gx, gy, gz;
                    int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
                    int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;


                    float comp_angle_x, comp_angle_y;
                    float acc_x_filtered, acc_y_filtered, acc_z_filtered;
                    float acc_roll, acc_pitch, gyro_roll, gyro_pitch, gyro_yaw, roll, pitch, yaw;
                    movingAvg mov_roll;
                    movingAvg mov_pitch;
//                    MedianFilter acc_x_filter, acc_y_filter, acc_z_filter;
                
                    uint32_t gyro_update_timer, accel_update_timer, combination_update_timer;
                    unsigned long timer = 0;
                    float timeStep = 0.01;
                    //A Matrix
                    float A[3][3] = { {0.996185f, 0.012151f, 0.001020f} ,
                                             {0.012151f, 0.999230f, -0.001506f} ,
                                             {0.001020f, -0.001506f, 0.983975f}
                                           };
                    //Bias Vector
                    float B[3] = {0.044777f, -0.000109f, -0.124497f};

                
};
#endif
