#include "main.h"
#include "encoder_control.h"
#include "jump_control.h"
#include "pwm_control.h"
#include <math.h>
#include "stm32f3xx_hal.h"
#include <stdio.h>
#include <string.h>

#define constrain(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))

volatile long countHall = 0;
int rw = 0;
int pulseCount = 0;
int rwPulse;

float speeds_filterold=0;
float positions=0;
double PI_pwm;

float speeds_filter;

void Code_Hall() {
    countHall++;  // Increment encoder pulse count
}

void countpulse()
{
    rw = countHall;  // Assign the value counted by the encoder to `rw`
    countHall = 0;   // Reset `countHall` for the next cycle

    pulseCount = rw;  

    // Determine pulse direction based on `pwmOut`
    if (pwmOut < 0)  
    {
        pulseCount = -pulseCount;  // Negative pulses if counterclockwise
    }
		else if (pwmOut > 0)    //Reaction wheel turning clockwise than pulse is a positive number.
		{
    pulseCount = pulseCount;
		}
    // Accumulate pulses every 5ms
    rwPulse += pulseCount;
}

// Speed control PI loop
void SpeedPIout() 
{
    // Calculate the speed from the encoder pulses (using a fixed conversion factor if needed)
    float speeds = rwPulse * 1.0;      // Speed from encoder pulses
    rwPulse = 0;                       // Reset encoder count for the next cycle

    // Apply a first-order complementary filter to smooth the speed reading
    speeds_filterold *= 0.7f;
    speeds_filter = speeds_filterold + speeds * 0.3f;
    speeds_filterold = speeds_filter;        // Store the filtered value for the next cycle

    // Integrate the speed to get the position (used for integral term in PI control)
    positions += speeds_filter;

    // Anti-windup: limit the position value to prevent runaway accumulation of integral error
    positions = constrain(positions, -3550, 3550);  

    PI_pwm = ki_speed * (targetAngle - positions) + kp_speed * (targetAngle - speeds_filter);
}
