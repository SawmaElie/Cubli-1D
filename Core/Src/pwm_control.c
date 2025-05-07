#include "main.h"
#include "pwm_control.h"
#include "jump_control.h"
#include "encoder_control.h"
#include <main.h>
#include <math.h>
#include "stm32f3xx_hal.h"
#include <stdio.h>
#include <string.h>

float angle0 = -5; // Mechanical balance angle
// Good YouTube video resource for PID's https://www.youtube.com/watch?v=0vqWyramGy8
////////////////////// Begin of PID parameters ///////////////////////////////
double kp = 28;
double ki = 0; // NOT USED
double kd = 0.62;
////////////////////// End of PID parameters /////////////////////////////////
///////////////////////////////// Begin of PID speed loop Vars //////////////////////
double kp_speed =  3; 
double ki_speed = 0.072;
double kd_speed = 0; // NOT USED  

double targetAngle = -5; // Angle balance point

int PD_pwm;  // Angle control output
float pwmOut = 0;
float pwmOut2 = 0;
///////////////////////////////// End of PID speed loop Vars //////////////////////

void PD()
{
    PD_pwm = kp * (angle + angle0) + kd * angle_speed;
}
