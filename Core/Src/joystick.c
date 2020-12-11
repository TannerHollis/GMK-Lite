/*
 * joystick.c
 *
 *  Created on: Dec 2, 2020
 *      Author: THollis
 */

#include "joystick.h"

joystick 	js[NUM_OF_JOYSTICKS];

uint32_t 	ADC_Buffer[2*NUM_OF_JOYSTICKS] 	= {0, 0};//Declare Analog buffer
uint8_t 	ADC_Channels[2*NUM_OF_JOYSTICKS] = {0, 1};//Declare Analog channels

/**
  * @brief  Initialize each Joystick
  * @note	Each joystick is initialized. The respsective channel according to each Joystick
  * 		is assigned and default values are placed. The min/max ADC values are set at opposite
  * 		ends to assure the joystick's ADC bounds are calculated properly.
  * @param  None
  * @retval None
  */
void Init_Joysticks(){
	for (uint8_t i = 0; i < NUM_OF_JOYSTICKS; i++){
		joystick *J = &js[i];
		J->ADC_CH[0] = ADC_Channels[i*2];
		J->ADC_CH[1] = ADC_Channels[i*2+1];
		J->ADC_Value[0] = 2048;
		J->ADC_Value[1] = 2048;
		J->ADC_Neutral[0] = 0;
		J->ADC_Neutral[1] = 0;
		J->ADC_count= 0;
		J->ADC_calibrate = 0;
		J->ADC_new_flag = 0;
		J->ADC_min[0] = UINT32_MAX;
		J->ADC_min[1] = UINT32_MAX;
		J->ADC_max[0] = 0;
		J->ADC_max[1] = 0;
		J->x = 0;
		J->y = 0;
	}
}

/**
  * @brief  Calibrate each Joystick
  * @note	Each joystick is calibrated using an averaging technique where the active joystick
  * 		is flagged for calibration. The joystick's respective ADC value is summed CALIBRATE_ITERS
  * 		times and then divided to to find the "neutral" resting position of the joystick.
  * 		This value is used later in the, map(), function. Where it maps the joystick's ADC value
  * 		to an output bounds required by the HID device.
  * @note	This function seems to have an infinite loop in the while statement as it relies on the
  * 		flag, ADC_new_flag, boolean. This boolean variable is triggered by the timer function,
  * 		HAL_TIM_PeriodElapsedCallback(), which calls the function, Update_Values(). In
  * 		Update_Values(), if the device that is being calibrated is flagged, ADC_calibrate, the
  * 		new value flag, ADC_new_flag, is set.
  * @param  None
  * @retval None
  */
void Calibrate_Joysticks(){
	for (uint8_t i = 0; i < NUM_OF_JOYSTICKS; i++){
		joystick *J = &js[i];
		J->ADC_calibrate = 1;
		J->ADC_count = 0;
		uint32_t sum[2] = {0, 0};
		while (J->ADC_count < CALIBRATE_ITERS){
			if (J->ADC_new_flag){
				sum[0] += J->ADC_Value[0];
				sum[1] += J->ADC_Value[1];
				J->ADC_new_flag = 0;
			}
		}
		J->ADC_Neutral[0] = sum[0] / CALIBRATE_ITERS;
		J->ADC_Neutral[1] = sum[1] / CALIBRATE_ITERS;
		J->ADC_calibrate = 0;
	}
}

/**
  * @brief  Update the Joystick's Max/Min Values
  * @note	The input Joystick is updated with it's newest ADC value, ADC_Value, where it sees if the
  * 		maximum value is less/more than the current value.
  * @note	This function is crutial to the Joystick's calibration functionality as the min/max
  * 		determine how far the joystick is either in the negative or positive direction.
  * @param  Joystick pointer J
  * @retval None
  */
void Update_Bounds(joystick *J){
	if (J->ADC_Value[0] > J->ADC_max[0]){
		J->ADC_max[0] = J->ADC_Value[0];
	}
	if (J->ADC_Value[0] < J->ADC_min[0]){
		J->ADC_min[0] = J->ADC_Value[0];
	}
	if (J->ADC_Value[1] > J->ADC_max[1]){
		J->ADC_max[1] = J->ADC_Value[1];
	}
	if (J->ADC_Value[1] < J->ADC_min[1]){
		J->ADC_min[1] = J->ADC_Value[1];
	}
}

/**
  * @brief  Map the Joystick's ADC Values for output
  * @note	The input Joystick's ADC value, ADC_Value, is mapped between two values as required by
  * 		the HID device.
  * @note	The ADC resolution is only 12-bits, while the HID device's output is 16-bits. In order to fake
  * 		a 16-bit resolution, either a basic linear interpolation or a more complex mapping function
  * 		is used. See the function, map(), for an in depth example.
  * @param  Joystick pointer J
  * @retval None
  */
void Map_Values(joystick *J){
	J->x = map(J->ADC_Value[0], J->ADC_min[0], J->ADC_max[0], INT16_MIN+4, INT16_MAX-3, J->ADC_Neutral[0]);
	if (INVERT_X)
		J->x = -(J->x);
	J->y = map(J->ADC_Value[1], J->ADC_min[1], J->ADC_max[1], INT16_MIN+4, INT16_MAX-3, J->ADC_Neutral[1]);
	if (INVERT_Y)
		J->y = -(J->y);
}

/**
  * @brief  Update each Joystick's ADC Values
  * @note	Updates each Joystick's ADC values with respect to the Joystick's ADC buffer location.
  * 		Since the ADC buffer, ADC_Buffer, is a physical location in ram, when an ADC conversion is
  * 		performed, it completes all channels at once. During this process, it will increment the RAM
  * 		address by 1 and move onto the next channel to convert. In order to keep track of the location
  * 		of each Joystick's respective ADC channel, each Joystick holds a structure member, ADC_CH.
  * @note	This function also checks to see if the current Joystick that is being updated is flagged
  * 		for calibration. If so, it increments the ADC calibration counts, ADC_count, and flags
  * 		for a new conversion using the, ADC_new_flag, boolean variable.
  * @param  None
  * @retval None
  */
void Update_Values(){
	for (uint8_t i = 0; i < NUM_OF_JOYSTICKS; i++){
		joystick *J = &js[i];
		J->ADC_Value[0] = ADC_Buffer[ J->ADC_CH[0] ];
		J->ADC_Value[1] = ADC_Buffer[ J->ADC_CH[1] ];
		if (J->ADC_calibrate){
			J->ADC_count++;
			J->ADC_new_flag = 1;
		}
	}
}

/**
  * @brief  Map the Input Value Between two Output Values
  * @note	Returns a value based on a simple interpolation or using a complex mapping function.
  * @note	Depending on whether or not calibration is enabled using, EN_CALIBRATE, the function will either
  * 		not process the input and do a simple interpolation. If the input bounds are zero, this function will
  * 		divide by zero, to protect from this error the difference is calculated and if found to be zero,
  * 		it returns zero.
  * @note	The complex mapping is done by taking the neutral "resting" position of the Joystick calculated
  * 		by the , Calibrate_Joystick(),  function. The difference between the input value, x, and the neutral
  * 		value is calculated to check if the Joystick is negative or positive. Once it determines the direction,
  * 		the result is calculated using a percentage of the ADC value from the ADC min/max and is mapped according
  * 		to the percentage of travel.
  * @note	Example: x = 400, neutral = 450, x_min = 0, x_max = 4095, y_min = -127, y_max = 127
  * 			(x - neutral) = -50, so negative
  * 			result = (x - neutral) * (avg_y - y_min) / (neutral - x_min) + avg_y
  * 			(x - neutral) / (neutral - x_min) = (400 - 450) / (450 - 0) = -11.11%
  * 			-11.11% * (avg_y - y_min) = -11.11% * (0 - -127) = -14.11
  * 			-14.11 + avg_y = -14.11 + 0 = -14.11
  * @note	Due to multiple integer divisions and multiplication, multiplcations are executed first to avoid unwanted
  * 		truncation of remainders as a byproduct of integer division. In order to avoid drift, the average of the
  * 		output bounds must be a whole number.
  * @param  int32_t x, int32_t x_min, int32_t x_max, int32_t y_min, int32_t y_max, int32_t neutral
  * @retval int32_t
  */
int32_t map(int32_t x, int32_t x_min, int32_t x_max, int32_t y_min, int32_t y_max, int32_t neutral){
	if (x_max - x_min == 0){
		return(0);
	}
	else{
		int32_t diff_x = x_max - x_min;
		int32_t diff_y = y_max - y_min;
		int32_t avg_y  = (y_max + y_min) / 2;
		int32_t result;
		if(EN_CALIBRATE){
			if(absolute(x - neutral) < DEADZONE){
				return(0);
			}
			else
			{
				if(x - neutral > 0){
					result = (x - neutral) * (y_max - avg_y) / (x_max - neutral) + avg_y;
				}
				else{
					result = (x - neutral) * (avg_y - y_min) / (neutral - x_min) + avg_y;
				}
			}
		}
		else{
			result = (x - x_min) * (diff_y) / (diff_x) + y_min;
		}

		result = result < y_min ? y_min : result;
		result = result > y_max ? y_max : result;
		return(result);
	}
}

/**
  * @brief  Absolute Value
  *
  * @param  int32_t value
  * @retval int32_t
  */
int32_t absolute(int32_t value){
	return (value < 0 ? -value : value);
}
