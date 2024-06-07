/*
 * motor_control.h
 *
 *  Created on: 2024 Jun 7
 *      Author: simeonidi03
 */
#include "at32f403a_407.h"
#include <stdio.h>

#ifndef INCLUDE_MOTOR_CONTROL_H_
#define INCLUDE_MOTOR_CONTROL_H_
#define CW  0 // clockwise         по часовой
#define CCW 1 // counterclock-wise против часовой

#define DIRECTION_SECOND_WHEEL_1 0 //pulse 1 or PA8
#define PWM_1 1                    //pulse 2 or PA9
#define DIRECTION_SECOND_WHEEL_2 2 //pulse 3 or PA10
#define PWM_2 3                    //pulse 4 or PA11

extern uint16_t timer_period;
extern uint16_t channel1_pulse, channel2_pulse, channel3_pulse, channel4_pulse;

typedef void (*pSetPIDMotor)(int16_t);

//структура с переменными для ПИД регулятора и управление мотором
typedef struct {
	char name; //имя двигателя, для отладки
	volatile uint8_t* timerPWMReg;		//регистр таймера
	volatile uint8_t* portCw; //порт выхода направления вращения
	volatile uint8_t* pinCw; //пин выхода направления вращения
	uint8_t bitCwMask; //маска
	volatile uint8_t* pinFg; //пин входа обратной связи (датчик холла)
	uint8_t bitFgMask; //маска
	//uint8_t dir; //направление вращения для задания через шину i2c
	int32_t odomCount; //счетчик тиков датчика холла
	int32_t odomCountOld; //старое значение тиков с датчика холла
	int16_t setParrot;	//уставка (тики датчика холла за время timePID)
	int16_t integralSum; //интегральная сумма
	int16_t oldErrorParrot;	//старое значение ошибки
	int16_t currentParrot; 	//текущие тики датчика хола за время timePID
	float pParrot; //пропоциональная составляющая
	float iParrot; //интегральная соствляющая
	float dParrot; //дифференциальная составляющая
	int16_t resPID; //сумма всех составляющих ПИД
	int16_t sendParrot; //двухбайтовая переменная для отправки данных по шине i2c
	pSetPIDMotor SetPIDMotor; //указатель на функцию задающую вращение мотора
} MotorData;

extern MotorData current_data;

void SetMotorDir(int16_t wheel_direction, char motor_number);



#endif /* INCLUDE_MOTOR_CONTROL_H_ */
