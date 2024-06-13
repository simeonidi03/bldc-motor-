#include "at32f403a_407_board.h"
#include "at32f403a_407_clock.h"
#include "at32f403a_407_wk_config.h"
#include <stdio.h>
#include "motor_control.h"

#define CW  0 // clockwise         по часовой
#define CCW 1 // counterclock-wise против часовой

WorkParams workParams;

MotorData motorA;
MotorData motorB;

uint16_t usart2_tx_buffer = 1;
uint8_t usart2_rx_buffer;
uint8_t usart3_rx_buffer;
uint8_t usart2_tx_buffer_size = 1;

void usart_configuration(void);
void usart2_tx_rx_handler(void);


int32_t odometr_div18 = 0;
int32_t odometr_bef = 0;
int32_t odometr_next = 0;
int16_t speed_hall = 0;

uint16_t direction = CW;

//	  gpio_bits_write(GPIOA, GPIO_PINS_4, CCW);

/** @addtogroup AT32F403A_periph_examples
 * @{
 */

/** @addtogroup 403A_TMR_7_pwm_output TMR_7_pwm_output
 * @{
 */

gpio_init_type gpio_init_struct = { 0 };
tmr_output_config_type tmr_output_struct;
crm_clocks_freq_type crm_clocks_freq_struct = { 0 };

uint16_t timer_period = 0;
uint16_t channel1_pulse = 0, channel2_pulse = 0, channel3_pulse = 0,
		channel4_pulse = 0;

/**
 * @brief  main function.
 * @param  none
 * @retval none
 */

void usart_configuration(void) {
	gpio_init_type gpio_init_struct;

	/* enable the usart2 and gpio clock */
	crm_periph_clock_enable(CRM_USART2_PERIPH_CLOCK, TRUE);
	crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);

	/* enable the usart3 and gpio clock */
	crm_periph_clock_enable(CRM_USART3_PERIPH_CLOCK, TRUE);
	crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

	gpio_default_para_init(&gpio_init_struct);

	/* configure the usart2 tx pin */
	gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
	gpio_init_struct.gpio_pins = GPIO_PINS_2;
	gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
	gpio_init(GPIOA, &gpio_init_struct);

	/* config usart nvic interrupt */
	nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
	nvic_irq_enable(USART2_IRQn, 0, 0);

	/* configure usart2 param */
	usart_init(USART2, 115200, USART_DATA_8BITS, USART_STOP_1_BIT);
	usart_transmitter_enable(USART2, TRUE);
	usart_receiver_enable(USART2, TRUE);

	/* enable usart2 and usart3 interrupt */
	usart_enable(USART2, TRUE);
	usart_interrupt_enable(USART2, USART_TDBE_INT, TRUE);

}

void holl_exint_init(void) {
	exint_init_type exint_init_struct;

	crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
	gpio_exint_line_config(GPIO_PORT_SOURCE_GPIOC, GPIO_PINS_SOURCE13);

	exint_default_para_init(&exint_init_struct);
	exint_init_struct.line_enable = TRUE;
	exint_init_struct.line_mode = EXINT_LINE_INTERRUPUT;
	exint_init_struct.line_select = EXINT_LINE_13;
	exint_init_struct.line_polarity = EXINT_TRIGGER_RISING_EDGE;
	exint_init(&exint_init_struct);

	nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
	nvic_irq_enable(EXINT15_10_IRQn, 0, 0);
}

void usart2_tx_rx_handler(void) {
	if (usart_flag_get(USART2, USART_TDBE_FLAG) != RESET) {
		/* write one byte to the transmit data register */
		// Передача старшего байта
		uint8_t high_byte = (uint8_t) (usart2_tx_buffer >> 8);
		usart_data_transmit(USART2, high_byte);

		// Передача младшего байта
		uint8_t low_byte = (uint8_t) usart2_tx_buffer;
		usart_data_transmit(USART2, low_byte);

		/* disable the usart2 transmit interrupt */
		usart_interrupt_enable(USART2, USART_TDBE_INT, FALSE);
	}
}

void usart2_tx_without_int() {
	int64_t odo_path = speed_hall;  // Примерное значение переменной
	char buffer[20] = { '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
			'0', '0', '0', '0', '0', '0', '0', '0', '0' };
	// Увеличиваем размер буфера для безопасного хранения длинных строк

	int8_t ostatok = 0;
	int8_t iter = 0;
	int8_t sight = 0;

	if (odo_path < 0) {
		sight = 0x4d;
		odo_path *= -1;
	}

	if (odo_path == 0) {
		buffer[iter++] = '0';
	} else {
		while (odo_path > 0) {
			ostatok = odo_path % 10;
			odo_path /= 10;
			buffer[iter++] = '0' + ostatok;  // Преобразование числа в символ
		}
	}

	if (sight) {
		buffer[iter] = '-';
		iter++;
	}

	buffer[iter] = '\r';
	buffer[++iter] = '\n';

	// Развернуть строку
	for (int i = 0; i < iter / 2; ++i) {
		char temp = buffer[i];
		buffer[i] = buffer[iter - i - 1];
		buffer[iter - i - 1] = temp;
	}
	//
	// Отправка строки через UART
	for (size_t i = 0; i <= iter; i++) {
		while (!USART2->sts_bit.tdbe)
			;
		usart_data_transmit(USART2, (int8_t) buffer[i]);
	}
}



int main(void) {
	system_clock_config();
	at32_board_init();

	/* get system clock */
	crm_clocks_freq_get(&crm_clocks_freq_struct);

	crm_init();
	wk_nvic_config();
	usart_configuration();
	wk_tmr6_init();
	wk_gpio_init();
	wk_tmr1_init();

	uint16_t speed = 750;

//  gpio_bits_write(GPIOA, GPIO_PINS_4, direction);
	channel2_pulse =
			(uint16_t) (((uint32_t) speed * (timer_period - 1)) / 1000);
	tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, channel2_pulse);

	holl_exint_init();

	MotorAInit(&motorA);

	while (1) {

		usart_data_transmit(USART2, usart2_tx_buffer);
		usart2_tx_buffer++;

		SetMotorDir(direction,1);
		//gpio_bits_write(GPIOA, GPIO_PINS_4, direction);
		//набор скорости
		for (int i = 0; i < 750; i++) {
			SetMotorPWM(&motorA ,speed);
			speed--;
			usart2_tx_without_int();
			delay_ms(50);
		}

		//сброс скорости
		for (int i = 0; i < 750; i++) {
			SetMotorPWM(&motorA ,speed);
			speed++;
			usart2_tx_without_int();
			delay_ms(50);
		}

		if (direction == CW) {
			direction = CCW;
		} else {
			direction = CW;
		}
		uint16_t speed = 750;
		at32_led_on(LED2);
	}
}

