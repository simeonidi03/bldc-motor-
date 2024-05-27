#include "at32f403a_407_board.h"
#include "at32f403a_407_clock.h"

#include <stdio.h>

#define CW  0 // clockwise         по часовой
#define CCW 1 // counterclock-wise против часовой

uint16_t usart2_tx_buffer = 1;
uint8_t usart2_rx_buffer;
uint8_t usart3_rx_buffer;
uint8_t usart2_tx_counter = 0x00;
uint8_t usart3_tx_counter = 0x00;
uint8_t usart2_rx_counter = 0x00;
uint8_t usart3_rx_counter = 0x00;
uint8_t usart2_tx_buffer_size = 1;

void usart_configuration(void);
void usart2_tx_rx_handler(void);


int16_t odometr = 0;
int32_t odometr_div18 = 0;

uint16_t direction = CW;

void usart_configuration(void);
void usart2_tx_rx_handler(void);

//	  gpio_bits_write(GPIOA, GPIO_PINS_4, CCW);

/** @addtogroup AT32F403A_periph_examples
  * @{
  */

/** @addtogroup 403A_TMR_7_pwm_output TMR_7_pwm_output
  * @{
  */

gpio_init_type  gpio_init_struct = {0};
tmr_output_config_type tmr_output_struct;
crm_clocks_freq_type crm_clocks_freq_struct = {0};

uint16_t timer_period = 0;
uint16_t channel1_pulse = 0, channel2_pulse = 0, channel3_pulse = 0, channel4_pulse = 0;

/**
  * @brief  main function.
  * @param  none
  * @retval none
  */


void usart_configuration(void)
{
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
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
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


void holl_exint_init(void)
{
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

void usart2_tx_rx_handler(void)
{
  if(usart_flag_get(USART2, USART_TDBE_FLAG) != RESET)
  {
    /* write one byte to the transmit data register */
	// Передача старшего байта
	uint8_t high_byte = (uint8_t)(usart2_tx_buffer >> 8);
	usart_data_transmit(USART2, high_byte);

	 // Передача младшего байта
	 uint8_t low_byte = (uint8_t)usart2_tx_buffer;
	 usart_data_transmit(USART2, low_byte);

    /* disable the usart2 transmit interrupt */
    usart_interrupt_enable(USART2, USART_TDBE_INT, FALSE);
  }
}



void usart2_tx_without_int() {
    int64_t odo_path = odometr_div18;  // Примерное значение переменной
    char buffer[20];  // Увеличиваем размер буфера для безопасного хранения длинных строк

    int8_t ostatok = 0;
    int8_t iter = 0;

    if (odo_path == 0) {
        buffer[iter++] = '0';
    } else {
        while (odo_path > 0) {
            ostatok = odo_path % 10;
            odo_path /= 10;
            buffer[iter++] = '0' + ostatok;  // Преобразование числа в символ
        }
    }

    buffer[iter] = '\0';

    // Развернуть строку
    for (int i = 0; i < iter / 2; ++i) {
        char temp = buffer[i];
        buffer[i] = buffer[iter - i - 1];
        buffer[iter - i - 1] = temp;
    }

    // Отправка строки через UART
    for (size_t i = 0; i < iter; i++) {
        usart_data_transmit(USART2, (int8_t)buffer[i]);
    }
}



int main(void)
{
  system_clock_config();

  at32_board_init();

  /* get system clock */
  crm_clocks_freq_get(&crm_clocks_freq_struct);

  /* enable tmr1/gpioa/gpiob clock */
  crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

  /* gpio output config */
  gpio_bits_set(GPIOA, GPIO_PINS_4);
  gpio_bits_reset(GPIOA, GPIO_PINS_5);

  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_pins = GPIO_PINS_4 | GPIO_PINS_5;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOA, &gpio_init_struct);


  /* timer1 output pin Configuration */
  gpio_init_struct.gpio_pins = /*GPIO_PINS_8 |*/ GPIO_PINS_9 | GPIO_PINS_10 | GPIO_PINS_11;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init(GPIOA, &gpio_init_struct);

  gpio_init_struct.gpio_pins = GPIO_PINS_13 | GPIO_PINS_14 | GPIO_PINS_15;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init(GPIOB, &gpio_init_struct);

  gpio_init_struct.gpio_pins = GPIO_PINS_12;
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init(GPIOB, &gpio_init_struct);

  gpio_init_struct.gpio_pins = GPIO_PINS_13;
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init(GPIOC, &gpio_init_struct);

  usart_configuration();



  /* tmr1 configuration generate 7 pwm signals with 4 different duty cycles:
   prescaler = 0, tmr1 counter clock = system_core_clock

   the objective is to generate 7 pwm signal at 17.57 khz:
     - tim1_period = (system_core_clock / 17570) - 1
   the channel 1 and channel 1n duty cycle is set to 50%
   the channel 2 and channel 2n duty cycle is set to 37.5%
   the channel 3 and channel 3n duty cycle is set to 25%
   the channel 4 duty cycle is set to 12.5%
   the timer pulse is calculated as follows:
     - channelxpulse = duty_cycle * (tim1_period - 1) / 100 */

  /* compute the value to be set in arr regiter to generate signal frequency at 20khz */
  timer_period = (crm_clocks_freq_struct.sclk_freq / 20000  ) - 1;

//  /* compute ccr1 value to generate a duty cycle at 50% for channel 1 and 1n */
//  channel1_pulse = (uint16_t)(((uint32_t) 5 * (timer_period - 1)) / 10);

  /* compute ccr2 value to generate a duty cycle at 37.5%  for channel 2 and 2n */
  channel2_pulse = (uint16_t)(((uint32_t) 375 * (timer_period - 1)) / 1000);

  /* compute ccr3 value to generate a duty cycle at 25%  for channel 3 and 3n */
  channel3_pulse = (uint16_t)(((uint32_t) 25 * (timer_period - 1)) / 100);

//  /* compute ccr4 value to generate a duty cycle at 12.5%  for channel 4 */
//  channel4_pulse = (uint16_t)(((uint32_t) 125 * (timer_period- 1)) / 1000);

  tmr_base_init(TMR1, timer_period, 0);
  tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);

  /* channel 1, 2, 3 and 4 configuration in output mode */
  tmr_output_default_para_init(&tmr_output_struct);
  tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_B;
  tmr_output_struct.oc_output_state = TRUE;
  tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_LOW;
  tmr_output_struct.oc_idle_state = TRUE;
  tmr_output_struct.occ_output_state = TRUE;
  tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.occ_idle_state = FALSE;

  /* channel 2 */
  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_2, &tmr_output_struct);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, channel2_pulse);

  /* channel 3 */
  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_3, &tmr_output_struct);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, channel3_pulse);

  /* output enable */
  tmr_output_enable(TMR1, TRUE);

  /* enable tmr1 */
  tmr_counter_enable(TMR1, TRUE);

  uint16_t speed = 750;


  gpio_bits_write(GPIOA, GPIO_PINS_4, direction);
  channel2_pulse = (uint16_t)(((uint32_t) speed * (timer_period - 1)) / 1000);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, channel2_pulse);

  holl_exint_init();


	while (1) {

		usart_data_transmit(USART2, usart2_tx_buffer);
		usart2_tx_buffer++;

		gpio_bits_write(GPIOA, GPIO_PINS_4, direction);
		//набор скорости
		for (int i = 0; i < 750; i++) {
			channel2_pulse = (uint16_t) (((uint32_t) speed * (timer_period - 1))
					/ 1000);
			tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, channel2_pulse);
			speed--;

			odometr = odometr_div18/10;
			usart2_tx_without_int();

//		    // Передача старшего байта
//		    uint8_t high_byte = (uint8_t)(odometr >> 8);
//		    usart_data_transmit(USART2, high_byte);
//		    // Передача младшего байта
//		    uint8_t low_byte = (uint8_t)odometr;
//		    usart_data_transmit(USART2, low_byte);

			delay_ms(50);
		}

		//сброс скорости
		for (int i = 0; i < 750; i++) {
			channel2_pulse = (uint16_t) (((uint32_t) speed * (timer_period - 1))
					/ 1000);
			tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, channel2_pulse);
			speed++;

			odometr = odometr_div18/10;
			usart2_tx_without_int();


//			odometr = odometr_div18/10;
//		    // Передача старшего байта
//		    uint8_t high_byte = (uint8_t)(odometr >> 8);
//		    usart_data_transmit(USART2, high_byte);
//		    // Передача младшего байта
//		    uint8_t low_byte = (uint8_t)odometr;
//		    usart_data_transmit(USART2, low_byte);

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


