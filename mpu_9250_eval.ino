#include <Wire.h>
#include "MPU_9250.h"

float ax, ay, az, gx, gy, gz, mx, my, mz;

float deltat = 0.0f, sum = 0.0f;  // integration interval for both filter schemes
uint32_t lastUpdate = 0;                        // used to calculate integration interval
uint32_t now = 0; 

MPU_9250 mpu9250;

void setup()
{
  Wire.begin();
  Serial.begin(57600);

  Serial.println("MPU_9250 starting calibration");
  mpu9250.calibrateMPU9250(mpu9250.gyroBias, mpu9250.accelBias);
  
  Serial.println("Initializing MPU_9250");
  mpu9250.initMPU9250();

  if(mpu9250.initAK8963(mpu9250.magCalibration))
  {
    Serial.println("Mag init was successful");
  }
  else
  {
    Serial.println("Mag init failed");
  }
}

void loop()
{
  imuRead();
  magRead();

  now = micros();
  deltat = ((now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = now;

  //MadgwickQuaternionUpdate(-ax, ay, az, gx*PI/180.0f, -gy*PI/180.0f, -gz*PI/180.0f, my, -mx, mz);
  //MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);

  Serial.print("rate = "); Serial.print(deltat, 4); Serial.println(" Hz");
}

void imuRead()
{
  if(mpu9250.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    int16_t imuData[6];
    
    mpu9250.readAccGyroData(imuData);

    ax = (float)imuData[0] / AFS_2G_SENS - mpu9250.accelBias[0];
    ay = (float)imuData[1] / AFS_2G_SENS - mpu9250.accelBias[1];
    az = (float)imuData[2] / AFS_2G_SENS - mpu9250.accelBias[2];

    gx = (float)imuData[3] * GFS_250_SENS;
    gy = (float)imuData[4] * GFS_250_SENS;
    gz = (float)imuData[5] * GFS_250_SENS;

    //Serial.println("Acc Data");
    //Serial.print(ax);Serial.print(" ");
    //Serial.print(ay);Serial.print(" ");
    //Serial.println(az);
    
    //Serial.println("Gyro Data");
    //Serial.print(gx);Serial.print(" ");
    //Serial.print(gy);Serial.print(" ");
    //Serial.println(gz);
  }
}

void magRead()
{
  int16_t imuData[3];
  
  if(mpu9250.readMagData(imuData))
  {
    mx = (float)imuData[0] * mpu9250.mRes * mpu9250.magCalibration[0] - mpu9250.magBias[0];
    mx *= mpu9250.magScale[0];

    my = (float)imuData[1] * mpu9250.mRes * mpu9250.magCalibration[1] - mpu9250.magBias[1];
    my *= mpu9250.magScale[1];

    mz = (float)imuData[2] * mpu9250.mRes * mpu9250.magCalibration[2] - mpu9250.magBias[2];
    mz *= mpu9250.magScale[2];
    
    //Serial.print(mx);Serial.print(",");
    //Serial.print(my);Serial.print(",");
    //Serial.println(mz);
  }
}

void magCalibration()
{
  mpu9250.calcMagCalibrationData(mpu9250.magBias, mpu9250.magScale);

  Serial.println("MagBias");
  Serial.print(mpu9250.magBias[0]);Serial.print("  ");
  Serial.print(mpu9250.magBias[1]);Serial.print("  ");
  Serial.println(mpu9250.magBias[2]);

  Serial.println("MagScale");
  Serial.print(mpu9250.magScale[0]);Serial.print("  ");
  Serial.print(mpu9250.magScale[1]);Serial.print("  ");
  Serial.println(mpu9250.magScale[2]);
}

void I2Cscan()
{
  // scan for i2c devices
  byte error, address;
  int nDevices;
  
  Serial.println("Scanning...");
  
  nDevices = 0;
  
  for(address = 1; address < 127; address++)
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if(error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if(address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");
      
      nDevices++;
    }
    else if(error == 4)
    {
      Serial.print("Unknow error at address 0x");
      if(address < 16) Serial.print("0");
      Serial.println(address,HEX);
    } 
    
  }
  
  if(nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}



