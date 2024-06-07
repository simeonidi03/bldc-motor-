/*
 * motor_control.c
 *
 *  Created on: 2024 Jun 7
 *      Author: @simeonidi03
 */
#include "motor_control.h"
#include "at32f403a_407.h"

void SetMotorDir(int16_t wheel_direction, char motor_number){

//CW  0 // clockwise         по часовой
//CCW 1 // counterclock-wise против часовой

if(motor_number == 1){
	if (wheel_direction) {
		/* channel 3 */
		channel3_pulse = (uint16_t) (((uint32_t) 1000 * (timer_period - 1))
				/ 1000);
		tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, channel3_pulse);
		//gpio_bits_reset(GPIOA, GPIO_PINS_4);
	} else {
		channel3_pulse =
				(uint16_t) (((uint32_t) 10 * (timer_period - 1)) / 1000);
		tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, channel3_pulse);
		//gpio_bits_set(GPIOA, GPIO_PINS_4);
	}
}else{
	if(motor_number == 1){
		if (wheel_direction) {
			/* channel 3 */
			channel1_pulse = (uint16_t) (((uint32_t) 1000 * (timer_period - 1))
					/ 1000);
			tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, channel1_pulse);
			//gpio_bits_reset(GPIOA, GPIO_PINS_4);
		} else {
			channel1_pulse =
					(uint16_t) (((uint32_t) 10 * (timer_period - 1)) / 1000);
			tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, channel1_pulse);
			//gpio_bits_set(GPIOA, GPIO_PINS_4);
		}
	}
}
}
