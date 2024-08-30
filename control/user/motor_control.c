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
			tmr_channel_value_set(TMR1, data->direction_channel,
					channel3_pulse);
			//gpio_bits_reset(GPIOA, GPIO_PINS_4);
		} else {
			channel3_pulse = (uint16_t) (((uint32_t) 10 * (timer_period - 1))
					/ 1000);
			tmr_channel_value_set(TMR1, data->direction_channel,
					channel3_pulse);
			//gpio_bits_set(GPIOA, GPIO_PINS_4);

		}
	} else if (data->name == 2) gpio_bits_write(GPIOA, GPIO_PINS_6, wheel_direction);

	data->direction = wheel_direction;
}

// от 0 до 750 есть смысл в регулировании
void SetMotorPWM(MotorData *data, uint16_t pwm) {

	if (pwm < workParams.pwmDeadZone) pwm = 0;

	uint16_t channel_pulse = (uint16_t) (((uint32_t) pwm * (timer_period - 1)) / 10000);
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
	workParams.limitSum = 150;					// лимит интегральной суммы
	workParams.timePID = 56;			//период ПИД регулятора
	workParams.pwmDeadZone = 40;		//мертвая зона ШИМ

}

void MotorAInit(MotorData *data){
	data->name = 1;
	data->channel_number = TMR_SELECT_CHANNEL_2;
	data->direction = CW;
	data->odomCountOld = 0;
	data->direction_channel = TMR_SELECT_CHANNEL_3;
}

void MotorBInit(MotorData *data){
	data->name = 2;
	data->channel_number = TMR_SELECT_CHANNEL_1;
	data->direction = CCW;
	data->odomCountOld = 0;
	data->direction_channel = TMR_SELECT_CHANNEL_4;
}

//вычисление ПИД регулятора для двигателя и установка направления вращения и ШИМ
void CalcPid(MotorData* data)
{

	int16_t errorParrot; //ошибка
	int32_t odomCount;
	int16_t setParrot;
	//uint16_t stepIntegralDecrement;


	odomCount = data->odomCount; //получаем значение счетчика одометра
	setParrot = 100 - (data->setParrot / 60); //получаем уставку
//	if(data->direction) setParrot *= -1;

	//текущие значения попугаев(импульсов датчика холла за время timePID)
	data->currentParrot = abs(odomCount - data->odomCountOld);
	data->odomCountOld = odomCount;
	errorParrot = setParrot - data->currentParrot; //ошибка

	data->integralSum += errorParrot;              //суммируем ошибку  (интегрирование)

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


	data->pParrot = workParams.kP * errorParrot;    // пропорциональная составляющая
	data->iParrot = workParams.kI * data->integralSum;    // интегральная составляющая
	data->dParrot = workParams.kD * (errorParrot - data->oldErrorParrot); //дифференциальная составляющая
	data->oldErrorParrot = errorParrot; //сохранил ошибку
	data->resPID = data->pParrot + data->iParrot + data->dParrot; //сумма ПИД

	if(data->resPID > 100) data->resPID = 100;
	if(data->resPID < 0 && data->direction) data->resPID = 0;


	SetMotorDirPWM(data, 6000 - 60 * abs(data->resPID)); //применил на мотор

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

	if (timerPWM > 6000)
		timerPWM = 6000;

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

void TMR3_DMA_Duty_Cycle(void)
{
	//for (int i = 0; i < 1000000; i++);

	/* set tmr channel CC value */
	tmr_channel_value_set(TMR3, TMR_SELECT_CHANNEL_4, 0);

  /* config set the number of data to be transferred by dma */
  dma_channel_enable(DMA1_CHANNEL3, FALSE);
  dma_data_number_set(DMA1_CHANNEL3, PWM_DataLength);
  dma_channel_enable(DMA1_CHANNEL3, TRUE);

	/* TMR enable counter */
	tmr_counter_enable(TMR3, TRUE);

	/* wait for the end of dma transfer */
	while (!dma_flag_get(DMA1_FDT3_FLAG));
	/* Clear dma flag */
	dma_flag_clear(DMA1_FDT3_FLAG);

	/* Clear TMR3 update Interrupt  pending bit */
	tmr_flag_clear(TMR3, TMR_OVF_FLAG);
  while(SET!=tmr_flag_get(TMR3, TMR_OVF_FLAG));
	/* Clear TMR3 update Interrupt  pending bit */
	tmr_flag_clear(TMR3, TMR_OVF_FLAG);
  while(SET != tmr_flag_get(TMR3, TMR_OVF_FLAG));

	/* TMR disable counter */
	tmr_counter_enable(TMR3, FALSE);

}


