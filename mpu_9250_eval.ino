#include <Wire.h>
#include "MPU_9250.h"

uint8_t rawData[6];  // x/y/z accel register data stored here
int16_t destination[3];

MPU_9250 mpu9250;

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  
  mpu9250.calibrateMPU9250(mpu9250.gyroBias, mpu9250.accelBias);
  mpu9250.initMPU9250();
  mpu9250.initAK8963(mpu9250.magCalibration);
}

void loop()
{
  if(mpu9250.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    
    mpu9250.readAccelData(destination);  // Read the six raw data registers into data array
  
    Serial.print((float)destination[0] / AFS_2G_SENS);Serial.print(" ");
    Serial.print((float)destination[1] / AFS_2G_SENS);Serial.print(" ");
    Serial.println((float)destination[2] / AFS_2G_SENS);
   
/*
    mpu9250.readGyroData(destination);

    Serial.print((float)destination[0] * GFS_250_SENS);Serial.print(" ");
    Serial.print((float)destination[1] * GFS_250_SENS);Serial.print(" ");
    Serial.println((float)destination[2] * GFS_250_SENS);
    */
  delay(500); 
  }
}



