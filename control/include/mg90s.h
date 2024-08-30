/*
 * mg90s.h
 *
 *  Created on: 2024 Mar 18
 *      Author: simeonidi03
 */

//in my certain case
#define NULL_SERVO 968
#define ONE_HUNDRED_SERVO 870

//in default case
#define NULL_SERVO_DEF 975
#define ONE_HUNDRED_SERVO_DEF 875




#ifndef INCLUDE_MG90S_H_
#define INCLUDE_MG90S_H_

uint16_t tm[8] = {};

uint16_t servo_pos[8] = {};

void servo_val_set(size_t value){
	if(value > 255 || value < 0){return;}

	uint16_t new_value = NULL_SERVO - ((NULL_SERVO - ONE_HUNDRED_SERVO_DEF + 5) * value) / 255;

	for(int i = 0; i < 8; ++i){
		servo_pos[i] = new_value;
	}
}

#endif /* INCLUDE_MG90S_H_ */
