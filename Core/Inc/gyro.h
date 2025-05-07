/*
 * gyro.h
 *
 *  Created on: Oct 5, 2023
 *      Author: Elie Sawma
 */

#ifndef SRC_GYRO_H_
#define SRC_GYRO_H_

//MOTOR PINS
#define MOTOR_GPIO_PORT GPIOB   // Motor control signals on GPIOB
#define MOTOR_PWM_PORT GPIOA    // PWM output on GPIOA
#define MOTOR_PWM_CHANNEL TIM_CHANNEL_1  // TIM1 CH1 (PA8)
#define MOTOR_START_STOP_PIN GPIO_PIN_1  // PB1
#define MOTOR_DIR_PIN GPIO_PIN_14        // PB14
#define ENCODER_PIN GPIO_PIN_12
#define ENCODER_GPIO_PORT MOTOR_GPIO_PORT  // Same as MOTOR_GPIO_PORT (GPIOB)


#include <stdint.h>
#include "stm32f3xx_hal.h"
#include <stdint.h>

#define READWRITE_CMD ((uint8_t)0x80)
#define MULTIPLEBYTE_CMD ((uint8_t)0x40)
#define CS_ON HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
#define CS_OFF HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
#define DUMMY_BYTE ((uint8_t)0x00)
#define PI 3.14159265359

void Error(void);
void Success(void);
void Gyro_IO_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);
void Gyro_IO_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
uint8_t Gyro_ReadID(void);
void Gyro_GetXYZ(int16_t* pData);
void sendGyroData(float X, float Y, float Z);
void Gyro_Calibrate(void);

// MPU6050 structure
typedef struct
{
    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;
	  int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

} MPU6050_t;




uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx);

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);
void MPU6050_Read_Accel (I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void angle_calculate(int16_t accX,int16_t accY,int16_t accZ,int16_t gyroX,int16_t gyroY,int16_t gyyroZ,float dt,float Q_angle,float Q_gyro,float R_angle,float C_0,float K1);
//float angle_calculate(I2C_HandleTypeDef *I2Cx);
void Kalman_Filter(float angle_m, float gyro_m);
void Yiorderfilter(float angle_m, float gyro_m);
#endif /* SRC_GYRO_H_ */

// Kalman Filter Variables
extern float Q_angle;    // Covariance of gyroscope noise    
extern float Q_gyro;     // Covariance of gyroscope drift noise
extern float R_angle;      // Covariance of accelerometer noise
extern char C_0;
extern float dt;         // Filter sampling time (5ms = 200Hz)
extern float K1;          // Kalman gain adjustment
extern float K_0, K_1, t_0, t_1;
extern float angle_err;
extern float q_bias;       // Gyroscope bias
extern float angle;        // Filtered angle
extern float angleY_one;
extern float angle_speed;
extern float Pdot[4];
extern float P[2][2];
extern float PCt_0, PCt_1, E;

