#include "main.h"
#include "jump_control.h"
#include "encoder_control.h"
#include "pwm_control.h"
#include <math.h>
#include "stm32f3xx_hal.h"
#include <stdio.h>
#include <string.h>
float loopOnce = 0;  // Used to reset Nidec jump-up routine


void ReactionWheelPWM()
{
    // Combine PD and PI control outputs
    pwmOut = -PD_pwm - PI_pwm;

    // Limit to -255 to 255
    if (pwmOut > 255) pwmOut = 255;
    if (pwmOut < -255) pwmOut = -255;

//    // Manually map -255..255 to -180..130 (motor-specific range)
//    pwmOut2 = ((pwmOut + 255.0f) * (130.0f + 180.0f)) / 510.0f - 180.0f;

    // Jump-up routine (right side)
    if (angle >= 25 && loopOnce == 1) 
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // Release brake (Start motor)
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // Direction CW
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);  // High-speed spin
        HAL_Delay(4000);

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // Direction CCW
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 170);    // Slow reverse
        pwmOut2 = 0;
        HAL_Delay(125);
        loopOnce = 0;
    }

    // Reset for jump-up (left side)
    if (angle <= -25 && loopOnce == 0) 
    {
        loopOnce = 1;
    }

    // Stop motor if too tilted
    if (angle >= 20 || angle <= -20) 
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // Stop
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 180);
    } 
    else 
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // Start

        if (pwmOut >= 0) 
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // CW
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwmOut);
        } 
        else 
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);   // CCW
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -pwmOut);
        }
    }
}
