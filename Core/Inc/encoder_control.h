#include <stdint.h>
#include "stm32f3xx_hal.h"
#include <stdint.h>
#include <math.h>

void Code_Hall(void);
void countpulse(void);
void SpeedPIout(void);

extern volatile long countHall;
extern int cc;
////////////////////// Begin of pulse count /////////////////////////
extern int rw;
extern int pulseCount;
extern int rwPulse;
////////////////////// End of pulse count //////////////////////////

//////////////////////////////// Begin of PI_pwm Vars //////////////////////////
extern float speeds_filterold;
extern float positions;
extern double PI_pwm;

extern float speeds_filter;
//////////////////////////////// End of PI_pwm Vars /////////////////////////////
