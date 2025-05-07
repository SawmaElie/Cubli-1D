/*
 * filters.h
 *
 *  Created on: Oct 5, 2023
 *      Author: Asus
 */

#ifndef FILTERS_H_
#define FILTERS_H_

#include "stm32f3xx_hal.h"

float Moving_Average_Filter(float newVal);
float Moving_Average_Filter1(float newVal);
float Moving_Average_Filter2(float newVal);

float Moving_Average_Filter3(float newVal);
float Moving_Average_Filter4(float newVal);
float Moving_Average_Filter5(float newVal);

#endif /* FILTERS_H_ */
