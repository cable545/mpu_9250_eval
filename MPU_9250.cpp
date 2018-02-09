#include "MPU_9250.h"

MPU_9250::MPU_9250(void)
{
  magBias[0] = MAG_BIAS_X;
  magBias[1] = MAG_BIAS_Y;
  magBias[2] = MAG_BIAS_Z;

  magScale[0] = MAG_SCALE_X;
  magScale[1] = MAG_SCALE_Y;
  magScale[2] = MAG_SCALE_Z;
}

// Wire.h read and write protocols
void MPU_9250::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

uint8_t MPU_9250::readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data;                            // `data` will store the register data   
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address 
  data = Wire.read();                      // Fill Rx buffer with result
  
  return data;                             // Return data read from slave register
}

void MPU_9250::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire.requestFrom(address, count);  // Read bytes from slave register address 
  
  while(Wire.available())
  {
    dest[i++] = Wire.read();       // Put read results in the Rx buffer
  }
}

void MPU_9250::getMres()
{
  switch(Mscale)
  {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
          mRes = 10.*4912./8190.; // Proper scale to return milliGauss
          break;
    case MFS_16BITS:
          mRes = 10.*4912./32760.0; // Proper scale to return milliGauss
          break;
  }
}

void MPU_9250::readAccelData(int16_t* destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}

void MPU_9250::readGyroData(int16_t* destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}

void MPU_9250::readAccGyroData(int16_t* destination)
{
  uint8_t rawData[14];
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 14, &rawData[0]);  // Read the fourteen raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;

  destination[3] = ((int16_t)rawData[8] << 8) | rawData[9] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[4] = ((int16_t)rawData[10] << 8) | rawData[11] ;
  destination[5] = ((int16_t)rawData[12] << 8) | rawData[13] ;
}

bool MPU_9250::readMagData(int16_t* destination)
{
  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of
  // data acquisition
  uint8_t rawData[7];
  // Wait for magnetometer data ready bit to be set
  if(readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01)
  {
    // Read the six raw data and ST2 registers sequentially into data array
    readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);
    uint8_t c = rawData[6]; // End data read by reading ST2 register
    // Check if magnetic sensor overflow set, if not then report data
    if(!(c & 0x08))
    {
      // Turn the MSB and LSB into a signed 16-bit value
      destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];
      // Data stored as little Endian 
      destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
      destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];

      return true;
    }
  }
  
  return false;
}

bool MPU_9250::initAK8963(float* destination)
{
  if(readByte(AK8963_ADDRESS, WHO_AM_I_AK8963) == 0x48)
  {
    // First extract the factory calibration for each magnetometer axis
    uint8_t rawData[3];  // x/y/z gyro calibration data stored here
    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
    delay(10);
    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
    delay(10);
    readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
    destination[0] =  (float)(rawData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
    destination[1] =  (float)(rawData[1] - 128)/256. + 1.;  
    destination[2] =  (float)(rawData[2] - 128)/256. + 1.; 
    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
    delay(10);
    // Configure the magnetometer for continuous read and highest resolution
    // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
    // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
    writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
    delay(10);

    getMres();

    return true;
  }
  else
  {
    return false;
  }
}

bool MPU_9250::initMPU9250()
{
  // wake up device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  delay(100); // Wait for all registers to reset 

  if(readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250) == 0x71)
  {
    writeByte(MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz

    // get stable time source
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
    delay(200); 
    
    /* GYROSCOPE CONFIG */

    // Configure Gyro and Thermometer
    // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
    // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
    // be higher than 1 / 0.0059 = 170 Hz
    // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
    // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
    //writeByte(MPU9250_ADDRESS, CONFIG, 0x03);  

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    //writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate 
                                    // determined inset in CONFIG above
 
    // Set gyroscope full scale range
    // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
    // c = c & ~0xE0; // Clear self-test bits [7:5] 
    c = c & ~0x02; // Clear Fchoice bits [1:0] 
    c = c & ~0x18; // Clear AFS bits [4:3]
    c = c | GFS_250DPS << 3; // Set full scale range for the gyro
    // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, GFS_250DPS << 3); // Write new GYRO_CONFIG value to register

    /* ACCELEROMETER CONFIG */
    
    // Set accelerometer full-scale range configuration
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, AFS_2G << 3); // Write new ACCEL_CONFIG register value

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x03); // Write new ACCEL_CONFIG2 register value
    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
    // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
    // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
    // can join the I2C bus and all can be controlled by the Arduino as master
    writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
    writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
    delay(100);

    return true;
  }
  else
  {
    return false;
  }
}

// Function which accumulates gyro and accelerometer data after device
// initialization. It calculates the average of the at-rest readings and then
// loads the resulting offsets into accelerometer and gyro bias registers.
void MPU_9250::calibrateMPU9250(float* gyroBias, float* accelBias)
{  
  int32_t accel_bias[3] = {0, 0, 0}, gyro_bias[3] = {0, 0, 0};
  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  uint8_t data[14]; // data array to hold accelerometer and gyro x, y, z, data
  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  delay(100); // Wait for all registers to reset
  
  writeByte(MPU9250_ADDRESS, CONFIG, 0x01);               // Set low-pass filter to 188 Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, AFS_16G << 3);  // Write new ACCEL_CONFIG register value
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x03);        // Write new ACCEL_CONFIG2 register value
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);
  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);           // Enable data ready (bit 0) interrupt

  readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[2]);
  readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[4]);
  
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[2] << 8) | data[3]);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[4] << 8) | data[5]);

  for(int i = 0; i < CALIBRATION_ROUNDS; i++)
  {
    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 14, data); // read data for averaging
    accel_bias[0] += ((int16_t)data[0] << 8) | data[1];
    accel_bias[1] += ((int16_t)data[2] << 8) | data[3];
    accel_bias[2] += ((int16_t)data[4] << 8) | data[5];

    gyro_bias[0] += ((int16_t)data[8] << 8) | data[9];
    gyro_bias[1] += ((int16_t)data[10] << 8) | data[11];
    gyro_bias[2] += ((int16_t)data[12] << 8) | data[13];
  }

  accel_bias[0] = (int32_t)accel_bias[0] / CALIBRATION_ROUNDS;
  accel_bias[1] = (int32_t)accel_bias[1] / CALIBRATION_ROUNDS;
  accel_bias[2] = (int32_t)accel_bias[2] / CALIBRATION_ROUNDS;

  gyro_bias[0] = (int32_t)gyro_bias[0] / CALIBRATION_ROUNDS;
  gyro_bias[1] = (int32_t)gyro_bias[1] / CALIBRATION_ROUNDS;
  gyro_bias[2] = (int32_t)gyro_bias[2] / CALIBRATION_ROUNDS;

  if(accel_bias[2] > 0L)
    accel_bias[2] -= AFS_16G_SENS;
  else
    accel_bias[2] += AFS_16G_SENS;

  for(int i = 0; i < 3; i++)
    if((accel_bias_reg[i] & mask)) mask_bit[i] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit

  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;
  
  // Push gyro biases to hardware registers
  writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
  
  accel_bias_reg[0] -= accel_bias[0]; // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= accel_bias[1];
  accel_bias_reg[2] -= accel_bias[2];
  
  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
}

void MPU_9250::calcMagCalibrationData(float* dest1, float* dest2)
{
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

  Serial.println("Mag Calibration: Wave device in a figure eight until done!");
  delay(4000);
  
  // shoot for ~thirty seconds of mag data
  if(Mmode == 0x02) sample_count = 240;  // at 8 Hz ODR, new mag data is available every 125 ms
  if(Mmode == 0x06) sample_count = 3000;  // at 100 Hz ODR, new mag data is available every 10 ms
  
  for(ii = 0; ii < sample_count; ii++)
  {
    readMagData(mag_temp);  // Read the mag data   
    
    for (int jj = 0; jj < 3; jj++)
    {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    
    if(Mmode == 0x02) delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
    if(Mmode == 0x06) delay(12);  // at 100 Hz ODR, new mag data is available every 10 ms
  }
  
  // Get hard iron correction
  mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
  mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
  mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

  dest1[0] = (float) mag_bias[0] * mRes * magCalibration[0];  // save mag biases in G for main program
  dest1[1] = (float) mag_bias[1] * mRes * magCalibration[1];
  dest1[2] = (float) mag_bias[2] * mRes * magCalibration[2];
  
  // Get soft iron correction estimate
  mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
  mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
  mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts
  
  float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
  avg_rad /= 3.0;
  
  dest2[0] = avg_rad/((float)mag_scale[0]);
  dest2[1] = avg_rad/((float)mag_scale[1]);
  dest2[2] = avg_rad/((float)mag_scale[2]);
  
  Serial.println("Mag Calibration done!");
}

