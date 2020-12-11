/*
 * joystick.h
 *
 *  Created on: Dec 2, 2020
 *      Author: THollis
 */

#ifndef INC_JOYSTICK_H_
#define INC_JOYSTICK_H_

#include "main.h"

#define HIGHBYTE(x) ((uint8_t) (x >> 8))
#define LOWBYTE(x) ((uint8_t) x)

#define CALIBRATE_ITERS 100
#define EN_CALIBRATE 1
#define DEADZONE 20		//DEADZONE is a function of the maximum ADC value, 4096, divided by two
						//due to the absolute function.
						//	DEADZONE / 2048 = DEADZONE%
						// 	DEADZONE = 20 , DEADZONE = 0.96%
						// 	DEADZONE = 50 , DEADZONE = 2.44%
						// 	DEADZONE = 100, DEADZONE = 4.88%
#define INVERT_X 0
#define INVERT_Y 1

#define NUM_OF_JOYSTICKS 1

typedef struct joystick{
	uint8_t ADC_CH[2];
	uint32_t ADC_Value[2];
	uint32_t ADC_Neutral[2];

	//Calibrate variables
	uint8_t ADC_count;
	uint8_t ADC_calibrate;
	uint8_t ADC_new_flag;

	//Joystick operation variables
	uint32_t ADC_min[2];
	uint32_t ADC_max[2];

	int16_t x;
	int16_t y;
} joystick;

void Init_Joysticks();

void Update_Bounds(joystick *J);

void Map_Values(joystick *J);

void Calibrate_Joysticks();

void Update_Values();

int32_t map(int32_t x, int32_t x_min, int32_t x_max, int32_t y_min, int32_t y_max, int32_t neutral);

int32_t absolute(int32_t value);

#endif /* INC_JOYSTICK_H_ */
