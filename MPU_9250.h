#include <Wire.h>
#include <Arduino.h>


//Magnetometer Registers
#define AK8963_ADDRESS   0x0C
#define WHO_AM_I_AK8963  0x00 // should return 0x48
#define INFO             0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L    0x03  // data
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

// acc and gyro registers
#define WHO_AM_I_AK8963  0x00 // should return 0x48
#define XG_OFFSET_H       0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L       0x14
#define YG_OFFSET_H       0x15
#define YG_OFFSET_L       0x16
#define ZG_OFFSET_H       0x17
#define ZG_OFFSET_L       0x18
#define SMPLRT_DIV        0x19
#define CONFIG            0x1A
#define GYRO_CONFIG       0x1B
#define ACCEL_CONFIG      0x1C
#define ACCEL_CONFIG      0x1C
#define ACCEL_CONFIG2     0x1D
#define FIFO_EN           0x23
#define I2C_MST_CTRL      0x24
#define INT_ENABLE        0x38
#define INT_STATUS        0x3A
#define ACCEL_XOUT_H      0x3B
#define ACCEL_XOUT_L      0x3C
#define ACCEL_YOUT_H      0x3D
#define ACCEL_YOUT_L      0x3E
#define ACCEL_ZOUT_H      0x3F
#define ACCEL_ZOUT_L      0x40
#define GYRO_XOUT_H       0x43
#define GYRO_XOUT_L       0x44
#define GYRO_YOUT_H       0x45
#define GYRO_YOUT_L       0x46
#define GYRO_ZOUT_H       0x47
#define GYRO_ZOUT_L       0x48
#define MPU9250_ADDRESS   0x68
#define USER_CTRL         0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1        0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2        0x6C
#define FIFO_COUNTH        0x72
#define FIFO_COUNTL        0x73
#define FIFO_R_W           0x74
#define WHO_AM_I_MPU9250  0x75 // Should return 0x71
#define XA_OFFSET_H        0x77
#define XA_OFFSET_L        0x78
#define YA_OFFSET_H        0x7A
#define YA_OFFSET_L        0x7B
#define ZA_OFFSET_H        0x7D
#define ZA_OFFSET_L        0x7E

#define SCALE_VALUE 0.00006103515f
#define AFS_2G 0
#define AFS_4G 1
#define AFS_8G 2
#define AFS_16G 3

#define AFS_2G_SENS   16384
#define AFS_4G_SENS   8192
#define AFS_8G_SENS   4096
#define AFS_16G_SENS  2048

#define GFS_250DPS  0
#define GFS_500DPS  1
#define GFS_1000DPS 2
#define GFS_2000DPS 3

#define GFS_250_SENS  250.0f / 32768.0f
#define GFS_500_SENS  500.0f / 32768.0f
#define GFS_1000_SENS 1000.0f / 32768.0f
#define GFS_2000_SENS 2000.0f / 32768.0f

#define CALIBRATION_ROUNDS 10000

class MPU_9250
{
  protected:
    enum Mscale {
      MFS_14BITS = 0, // 0.6 mG per LSB
      MFS_16BITS      // 0.15 mG per LSB
    };
  
    uint8_t Mscale = MFS_16BITS;
    // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
    uint8_t Mmode = 0x02;
  
	public:
    // Bias corrections for gyro and accelerometer
    float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};
    float magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};
    float aRes, gRes, mRes;

    bool initAK8963(float* destination);
    bool initMPU9250();
    void calibrateMPU9250(float* gyroBias, float* accelBias);
		void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
		uint8_t readByte(uint8_t address, uint8_t subAddress);
		void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
    void getMres();
    void readAccelData(int16_t* destination);
    void readGyroData(int16_t* destination);
    void readMagData(int16_t * destination);
    void calcMagCalibrationData(float* dest1, float* dest2);
};
