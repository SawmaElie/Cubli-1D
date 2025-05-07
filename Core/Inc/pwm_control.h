#include <stdint.h>
#include "stm32f3xx_hal.h"
#include <stdint.h>
#include <math.h>

void PD(void);
extern int PD_pwm;
extern float pwmOut;
extern float pwmOut2;
extern float angle;
extern float angle_speed;
extern double kp_speed; 
extern double ki_speed;
extern double kd_speed; // NOT USED  
extern float angle0;
extern double targetAngle; // Angle balance point
extern double kp;
extern double ki;
extern double kd;
