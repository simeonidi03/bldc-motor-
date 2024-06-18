/*
 * motor_control.h
 *
 *  Created on: 2024 Jun 7
 *      Author: simeonidi03
 */
#include "at32f403a_407.h"
#include <stdio.h>
#include <math.h>
extern void* motorA_ptr;
extern void* motorB_ptr;

#ifndef INCLUDE_MOTOR_CONTROL_H_
#define INCLUDE_MOTOR_CONTROL_H_
#define CW  0 // clockwise         по часовой
#define CCW 1 // counterclock-wise против часовой
#define PWM_K 2.94 // перевод от системы от 0 до 255 в систему от 0 до 750

#define DIRECTION_SECOND_WHEEL_1 0 //pulse 1 or PA8
#define PWM_1 1                    //pulse 2 or PA9
#define DIRECTION_SECOND_WHEEL_2 2 //pulse 3 or PA10
#define PWM_2 3                    //pulse 4 or PA11

//режимы работы моторов
#define MOTOR_MODE_NONE 0 //моторы не крутим
#define MOTOR_MODE_PWM 1 //прямое управление моторами задавая скважность ШИМ
#define MOTOR_MODE_PID 2 //управление посредством ПИД регулятора

// Определение макросов для параметров преобразования
#define MIN_OLD 750.0
#define MAX_OLD 0.0
#define MIN_NEW 0.0
#define MAX_NEW 95.0


extern uint16_t timer_period;
extern uint16_t channel1_pulse, channel2_pulse, channel3_pulse, channel4_pulse;

typedef void (*pSetPIDMotor)(int16_t);

//------параметры сохраняемые в EEPROM----------------------------
typedef struct {
	uint8_t workMode; //режим работы
	float kP;  							// пропорциональный коэффициент
	int8_t kPint;
	float kI;  							// интегральный  коэффициент
	int8_t kIint;
	float kD;  							// дифференциальный  коэффициент
	int8_t kDint;
	int16_t limitSum;					// ограничение интегральной суммы
	uint16_t timePID;					// период работы ПИД регулятора в мс
	uint8_t pwmDeadZone;				//мертвая зона ШИМ, ниже которой значение будет всегда 0
} WorkParams;

//структура с переменными для ПИД регулятора и управление мотором
typedef struct {
	uint8_t name; //имя двигателя, для отладки
	uint8_t channel_number; // номер канала
	volatile uint8_t* portCw; //порт выхода направления вращения
	volatile uint8_t* pinCw; //пин выхода направления вращения
	uint8_t bitCwMask; //маска
	volatile uint8_t* pinFg; //пин входа обратной связи (датчик холла)
	uint8_t bitFgMask; //маска
	uint8_t direction; //направление вращения для задания через шину i2c
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

extern WorkParams workParams;


void SetMotorDir(int16_t wheel_direction, MotorData *data);
void SetMotorPWM(MotorData *data, uint16_t pwm);
void MotorAInit();
void PidParamInit();
void MotorBInit();
void CalcPid(MotorData* data);
void OdometrProcess(MotorData *data);
void CalcParrot(MotorData *data);
void SetMotorDirPWM(MotorData *data, int16_t pwm);

#endif /* INCLUDE_MOTOR_CONTROL_H_ */
