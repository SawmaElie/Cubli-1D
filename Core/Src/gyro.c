// gyro.c


#include "gyro.h"
#include <main.h>
#include <math.h>
#include "stm32f3xx_hal.h"
#include <stdio.h>
#include <string.h>

#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43

#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41

// Setup MPU6050
#define MPU6050_ADDR 0xD0

const uint16_t i2c_timeout = 100;
extern I2C_HandleTypeDef hi2c1;
#define FS_250DPS_SENSITIVITY 0.00875  // Sensitivity for 250 dps in dps/digit


void Error(){
	//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
}

void Success(){
	//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
}

//-------------Reading data from MPU6050-----------//
// Kalman Filter Variables
float Q_angle = 0.001;    // Covariance of gyroscope noise    
float Q_gyro = 0.003;     // Covariance of gyroscope drift noise
float R_angle = 0.5;      // Covariance of accelerometer noise
char C_0 = 1;
float dt = 0.005;         // Filter sampling time (5ms = 200Hz)
float K1 = 0.05;          // Kalman gain adjustment
float K_0, K_1, t_0, t_1;
float angle_err;
float q_bias = 0.0;       // Gyroscope bias
float angle = 0.0;        // Filtered angle
float angleY_one;
float angle_speed;
float Pdot[4] = { 0, 0, 0, 0 };
float P[2][2] = {{ 1, 0 }, { 0, 1 }};
float PCt_0, PCt_1, E;

// Function to initialize MPU6050

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx)
{
    uint8_t check;
    uint8_t Data;

    // check device ID WHO_AM_I

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, i2c_timeout);

    if (check == 104) // 0x68 will be returned by the sensor if everything goes well
    {
        // power management register 0X6B we should write all 0's to wake the sensor up
        Data = 0;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data = 0x07;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, i2c_timeout);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, i2c_timeout);
			
				// Set accelerometer configuration in ACCEL_CONFIG Register
				Data = 0x00;  // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ± 2g
				HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1C, 1, &Data, 1, i2c_timeout);
        return 0;
    }
    return 1;
}

//---------Reading Data from the Gyro--------//

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from GYRO_XOUT_H register
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

//    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
//    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
//    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;
}

void MPU6050_Read_Accel (I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H (0x3B) register
	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, 0x3B, 1, Rec_Data, 6, 1000);

	DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

//	DataStruct->Ax = DataStruct->Accel_X_RAW/16384.0;
//	DataStruct->Ay = DataStruct->Accel_Y_RAW/16384.0;
//	DataStruct->Az = DataStruct->Accel_Z_RAW/16384.0;
}

//-------------End of Gyroscope GY-521-----------//



void Kalman_Filter(float angle_m, float gyro_m) 
{
    angle += (gyro_m - q_bias) * dt;          // Prior estimate
    angle_err = angle_m - angle;
    
    Pdot[0] = Q_angle - P[0][1] - P[1][0];    // Covariance differential
    Pdot[1] = -P[1][1];
    Pdot[2] = -P[1][1];
    Pdot[3] = Q_gyro;
    
    P[0][0] += Pdot[0] * dt;
    P[0][1] += Pdot[1] * dt;
    P[1][0] += Pdot[2] * dt;
    P[1][1] += Pdot[3] * dt;
    
    // Compute Kalman Gain
    PCt_0 = C_0 * P[0][0];
    PCt_1 = C_0 * P[1][0];
    E = R_angle + C_0 * PCt_0;
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;
    
    t_0 = PCt_0;
    t_1 = C_0 * P[0][1];
    
    // Update covariance
    P[0][0] -= K_0 * t_0;
    P[0][1] -= K_0 * t_1;
    P[1][0] -= K_1 * t_0;
    P[1][1] -= K_1 * t_1;
    
    // Update estimates
    q_bias += K_1 * angle_err;
    angle_speed = gyro_m - q_bias;  
    angle += K_0 * angle_err;  
}

void Yiorderfilter(float angle_m, float gyro_m)
{
  angleY_one = K1 * angle_m + (1 - K1) * (angleY_one + gyro_m * dt);
}


//float angle_calculate(I2C_HandleTypeDef *I2Cx) 
//{
//    MPU6050_t sensorData;
//    
//    // Read accelerometer and gyroscope data
//    MPU6050_Read_Accel(I2Cx, &sensorData);
//    MPU6050_Read_Gyro(I2Cx, &sensorData);
//    
//    // Convert accelerometer data to angle (X-axis tilt)
//    float accel_angle = -atan2(sensorData.Ay, sensorData.Az) * (180.0 / PI);
//    
//    // Convert gyroscope data to degrees per second
//    float gyro_rate = sensorData.Gx;
//    
//    // Apply Kalman filter
//    Kalman_Filter(accel_angle, gyro_rate);
//    
//    return angle;  // Return the filtered angle
//}

////////////mpu6050////////////////////
int16_t GyroData[3];
int16_t AccelData[3];
float gyroX, gyroY, gyroZ;
float accelX, accelY, accelZ;
float Gyro_x,Gyro_y,Gyro_z;

MPU6050_t MPU6050;
////////////END mpu6050////////////////////

// Function to compute tilt angle
//float angle_calculate(I2C_HandleTypeDef *I2Cx)
//{
//    MPU6050_t sensorData;
//    // Read accelerometer and gyroscope data
//    MPU6050_Read_Accel(I2Cx, &sensorData);
//    MPU6050_Read_Gyro(I2Cx, &sensorData);

//    // Convert accelerometer data to angle (X-axis tilt)
//    float Angle = -atan2(sensorData.Accel_Y_RAW, sensorData.Accel_Z_RAW) * (180.0 / PI);

//    // Convert gyroscope data to degrees per second
//    float Gyro_x = -(sensorData.Gyro_X_RAW) / 131.0;
//    float Gyro_y = -(sensorData.Gyro_Y_RAW) / 131.0;
//    float Gyro_z = -(sensorData.Gyro_Z_RAW) / 131.0;

//    // Apply Kalman filter
//    Kalman_Filter(Angle, Gyro_x);

//    // Apply first-order filter
//    float angleAx = -atan2(sensorData.Accel_X_RAW, sensorData.Accel_Z_RAW) * (180.0 / PI);
//    Yiorderfilter(angleAx, Gyro_y);

//    return angle;  // Return the filtered tilt angle
//}

void angle_calculate(int16_t accX,int16_t accY,int16_t accZ,int16_t gyroX,int16_t gyroY,int16_t gyroZ,float dt,float Q_angle,float Q_gyro,float R_angle,float C_0,float K1)
{
  float Angle = -atan2(accY , accZ) * (180/ PI);           //Radial rotation angle calculation formula ; negative sign is direction processing
  Gyro_x = -gyroX / 131;              //The X-axis angular velocity calculated by the gyroscope;  the negative sign is the direction processing
  Kalman_Filter(Angle, Gyro_x);            //Kalman Filter
  //rotating angle Z-axis parameter
  Gyro_z = -gyroZ / 131;                      //angle speed of Z-axis
  //accelz = accZ / 1604;

  float angleAx = -atan2(accX, accZ) * (180 / PI); //calculate the inclined angle with x-axis
  Gyro_y = -gyroY / 131.00; //angle speed of Y-axis
  Yiorderfilter(angleAx, Gyro_y); //first-order filtering
}
////////////////////////////////////////////////////////////////
