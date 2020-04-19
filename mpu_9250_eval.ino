#include <Wire.h>
#include "MPU_9250.h"
#include "quaternionFilters.h"

#define DEG_TO_RAD PI / 180.0f
#define RAD_TO_DEG 180.f / PI

float ax, ay, az, gx, gy, gz, mx, my, mz;

float roll, pitch, yaw;

float deltat = 0.0f, sum = 0.0f;  // integration interval for both filter schemes
uint32_t lastUpdate = 0;                        // used to calculate integration interval
uint32_t now = 0; 

MPU_9250 mpu9250;

void setup()
{
  Wire.begin();
  Serial.begin(57600);

//  Serial.println("MPU_9250 starting calibration");
//  mpu9250.calibrateMPU9250(mpu9250.gyroBias, mpu9250.accelBias);
  
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
  //magRead();

  /*
  now = micros();
  deltat = ((now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = now;

  //MadgwickQuaternionUpdate(ax, ay, az, gx * DEG_TO_RAD, gy * DEG_TO_RAD, gz * DEG_TO_RAD, my, mx, mz, deltat);
  MahonyQuaternionUpdate(ax, ay, az, gx * DEG_TO_RAD, gy * DEG_TO_RAD, gz * DEG_TO_RAD, my, mx, mz, deltat);
  //MadgwickUpdate(ax, ay, az, gx * DEG_TO_RAD, gy * DEG_TO_RAD, gz * DEG_TO_RAD, deltat);

  yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() *
                    *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1)
                    - *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3));
  pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ() *
                    *(getQ()+2)));
  roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) *
                    *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1)
                    - *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));
  pitch *= RAD_TO_DEG;
  yaw   *= RAD_TO_DEG;
      //   5.75° W  ± 0.37° (or 5.75°) on 2018-02-09
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
  yaw   -= 5.75;
  roll *= RAD_TO_DEG;
  
  Serial.print("Orientation: ");
  Serial.print(roll);Serial.print(" ");
  Serial.print(pitch);Serial.print(" ");
  Serial.print(yaw); Serial.println("");
  */
}

void imuRead()
{
  if(mpu9250.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    int16_t imuData[6];
    
    mpu9250.readAccGyroData(imuData);

    ax = (float)imuData[0] / AFS_2G_SENS;
    ay = (float)imuData[1] / AFS_2G_SENS;
    az = (float)imuData[2] / AFS_2G_SENS;

    gx = (float)imuData[3] * GFS_250_SENS;
    gy = (float)imuData[4] * GFS_250_SENS;
    gz = (float)imuData[5] * GFS_250_SENS;

    //Serial.println("Acc Data");
    Serial.print(ax, 4);Serial.print(" ");
    Serial.print(ay, 4);Serial.print(" ");
    Serial.println(az, 4);
    
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



