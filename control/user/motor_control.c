/*
 * motor_control.c
 *
 *  Created on: 2024 Jun 7
 *      Author: @simeonidi03
 */
#include "motor_control.h"
#include "at32f403a_407.h"

// Функция для преобразования уровня работы двигателя из одной системы в другую
float convertEngineLevel(float oldLevel) {
    return ((MAX_OLD - oldLevel) * (MAX_NEW - MIN_NEW) / (MAX_OLD - MIN_OLD)) + MIN_NEW;
}

void SetMotorDir(int16_t wheel_direction, MotorData *data) {

//CW  0 // clockwise         по часовой
//CCW 1 // counterclock-wise против часовой
//  gpio_bits_write(GPIOA, GPIO_PINS_4, wheel_direction);
	if (data->name == 1) {
		if (wheel_direction) {
			/* channel 3 */
			channel3_pulse = (uint16_t) (((uint32_t) 1000 * (timer_period - 1))
					/ 1000);
			tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, channel3_pulse);
			//gpio_bits_reset(GPIOA, GPIO_PINS_4);
		} else {
			channel3_pulse = (uint16_t) (((uint32_t) 10 * (timer_period - 1))
					/ 1000);
			tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, channel3_pulse);
			//gpio_bits_set(GPIOA, GPIO_PINS_4);

		}
	} else {
		if (data->name == 2) {
			if (wheel_direction) {
				/* channel 3 */
				channel1_pulse = (uint16_t) (((uint32_t) 1000
						* (timer_period - 1)) / 1000);
				tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1,
						channel1_pulse);
				//gpio_bits_reset(GPIOA, GPIO_PINS_4);
			} else {
				channel1_pulse =
						(uint16_t) (((uint32_t) 10 * (timer_period - 1)) / 1000);
				tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1,
						channel1_pulse);
				//gpio_bits_set(GPIOA, GPIO_PINS_4);
			}
		}
	}
	data->direction = wheel_direction;
}

// от 0 до 750 есть смысл в регулировании
void SetMotorPWM(MotorData *data, uint16_t pwm) {

	if (pwm < workParams.pwmDeadZone) pwm = 0;

	uint16_t channel_pulse = (uint16_t) (((uint32_t) pwm * (timer_period - 1)) / 1000);
	tmr_channel_value_set(TMR1, data->channel_number, channel_pulse);

#ifdef MOTOR_INV
	pwm = ~pwm; //инвертируем PWM для моторов 25D
#endif

}


void PidParamInit() {
	//задаем режим работы
	workParams.workMode = MOTOR_MODE_NONE;

	//параметры ПИД регулятора по умолчанию
	workParams.kP = 0.5;  							// пропорциональный коэфф
	workParams.kPint = 5;
	workParams.kI = 2;  							// интегральный коэфф
	workParams.kIint = 20;
	workParams.kD = 0;  							// дифф коэфф
	workParams.kDint = 0;
	workParams.limitSum = 500;					// лимит интегральной суммы
	workParams.timePID = 56;			//период ПИД регулятора
	workParams.pwmDeadZone = 40;		//мертвая зона ШИМ

}

void MotorAInit(MotorData *data){
	data->name = 1;
	data->channel_number = 2;
	data->direction = CW;
	data->odomCountOld = 0;
}

//вычисление ПИД регулятора для двигателя и установка направления вращения и ШИМ
void CalcPid(MotorData* data)
{
	int16_t errorParrot; //ошибка
	int32_t odomCount;
	int16_t setParrot;
	//uint16_t stepIntegralDecrement;


	odomCount = data->odomCount; //получаем значение счетчика одометра
	setParrot = 100 - (data->setParrot / 7.89); //получаем уставку
	if(data->direction ) setParrot *= -1;

	data->currentParrot = odomCount - data->odomCountOld;		//текущие значения попугаев(импульсов датчика холла за время timePID)
	data->odomCountOld = odomCount;

	errorParrot = setParrot - data->currentParrot; //ошибка

	data->integralSum += errorParrot;          //суммируем ошибку  (интегрирование)

	/*
	Parms->integralBuff[Parms->integralCount] = errorParrot;
	Parms->integralCount++;
	if (Parms->integralCount > (IntegralBuffelLength-1))
		Parms->integralCount = 0;


	Parms->integralSum = 0;
	for (uint8_t i = 0; i<IntegralBuffelLength; i++)
		Parms->integralSum += Parms->integralBuff[i];
	*/

	if (data->integralSum > workParams.limitSum) //нормализуем интегральную сумму
		data->integralSum = workParams.limitSum;
	else
	{
		if (data->integralSum < -workParams.limitSum)
			data->integralSum = -workParams.limitSum;
	}

	//чтобы при заданной остановке робота интегральная сумма не влияла на обороты
	//уменьшаем ёё до 0
	if ((setParrot == 0) && (data->currentParrot == 0) && (data->integralSum != 0))
	{
		if (data->integralSum > 0)
			data->integralSum --;
		else
			if (data->integralSum < 0)
				data->integralSum ++;
	}

//	if ((data->setParrot == 0) && (data->currentParrot == 0) && (data->integralSum != 0))
//	{
//		stepIntegralDecrement = workParams.timePID >> 2;
//		if (stepIntegralDecrement == 0)
//			stepIntegralDecrement = 1;
//
//		if (data->integralSum > 0)
//		{
//			if (data->integralSum > stepIntegralDecrement)
//				data->integralSum -= stepIntegralDecrement;
//			else
//				data->integralSum = 0;
//		}
//		else
//		{
//			if (data->integralSum < -stepIntegralDecrement)
//				data->integralSum += stepIntegralDecrement;
//			else
//				data->integralSum = 0;
//		}
//	}

	data->pParrot = workParams.kP * errorParrot;    // пропорциональная составляющая
	data->iParrot = workParams.kI * data->integralSum;    // интегральная составляющая
	data->dParrot = workParams.kD * (errorParrot - data->oldErrorParrot); //дифференциальная составляющая
	data->oldErrorParrot = errorParrot; //сохранил ошибку
	data->resPID = data->pParrot + data->iParrot + data->dParrot; //сумма ПИД

	if(data->resPID > 100) data->resPID = 100;
	if(data->resPID < -100 && data->direction) data->resPID = 100;
	if(data->resPID > 100 && data->direction) data->resPID = 0;
	SetMotorDirPWM(data, 750 - 7.5 * abs(data->resPID)); //применил на мотор

//	printf_P(PSTR("odo:%d set:%d curr:%d P:%.2f I:%.2f D:%.2f res: %d\n"), (int)odomCount, (int)setParrot, (int)motorA.currentParrot,
//			(float)motorA.pParrot, (float)motorA.iParrot, (float)motorA.dParrot, (int)motorA.resPID);

}

void OdometrProcess(MotorData *data)
{
	if (data->direction == 0) //смотрим в какую сторону крутиться мотор
	{
		data->odomCount++; //увеличиваем значение одометра
	}
	else
	{
		data->odomCount--; //уменьшаем значение одометр
	}
}


void SetMotorDirPWM(MotorData *data, int16_t pwm)
{
	uint16_t timerPWM = abs(pwm);

	if (timerPWM > 750)
		timerPWM = 750;

	SetMotorDir(/*(pwm > 0)*/data->direction, data);
	SetMotorPWM(data, timerPWM);
}

void CalcParrot(MotorData *data)
{
	int32_t odomCount;
	odomCount = data->odomCount; //получаем значение одометра(тики датчика холла)
	data->currentParrot = odomCount - data->odomCountOld;//вычисляем количество тиков за время timePID (попугаи)
	data->odomCountOld = odomCount;
}




