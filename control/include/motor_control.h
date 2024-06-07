/*
 * motor_control.h
 *
 *  Created on: 2024 Jun 7
 *      Author: user
 */

#ifndef INCLUDE_MOTOR_CONTROL_H_
#define INCLUDE_MOTOR_CONTROL_H_


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





#endif /* INCLUDE_MOTOR_CONTROL_H_ */
