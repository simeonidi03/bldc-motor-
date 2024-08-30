/*
 * at32f403a_407_wk_config.c
 *
 *  Created on: 2024 Jun 6
 *      Author: simeonidi03
 */

#include "at32f403a_407_wk_config.h"

void wk_nvic_config(void)
{
  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
  nvic_irq_enable(TMR6_GLOBAL_IRQn, 1, 0);
  nvic_irq_enable(EXINT0_IRQn, 1, 0);
  nvic_irq_enable(I2C2_EVT_IRQn, 1, 0);
}

void wk_tmr6_init(void)
{
  /* add user code begin tmr6_init 0 */

  /* add user code end tmr6_init 0 */

  /* add user code begin tmr6_init 1 */

  /* add user code end tmr6_init 1 */

  /* configure counter settings */
  tmr_base_init(TMR6, 65535, 150);
  tmr_cnt_dir_set(TMR6, TMR_COUNT_UP);
  tmr_period_buffer_enable(TMR6, FALSE);

  /* configure primary mode settings */
  tmr_primary_mode_select(TMR6, TMR_PRIMARY_SEL_OVERFLOW);

  tmr_counter_enable(TMR6, TRUE);

  tmr_interrupt_enable(TMR6, TMR_OVF_INT, TRUE);

  /**
   * Users need to configure TMR6 interrupt functions according to the actual application.
   * 1. Call the below function to enable the corresponding TMR6 interrupt.
   *     --tmr_interrupt_enable(...)
   * 2. Add the user's interrupt handler code into the below function in the at32f403a_407_int.c file.
   *     --void TMR6_GLOBAL_IRQHandler(void)
   */

  /* add user code begin tmr6_init 2 */

  /* add user code end tmr6_init 2 */
}

void crm_init(void){

  /* enable tmr1/gpioa/gpiob clock */
  crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_TMR6_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);

	/* dma clock enable */
  crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
}


/*init ports */
void wk_gpio_init(void) {

	/* gpio output config */

	gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
	gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
	gpio_init_struct.gpio_pins = GPIO_PINS_6;
	gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
	gpio_init(GPIOA, &gpio_init_struct);

	/* timer1 output pin Configuration */
	gpio_init_struct.gpio_pins = GPIO_PINS_8 | GPIO_PINS_9 | GPIO_PINS_10 | GPIO_PINS_11;
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

	gpio_init_type gpio_init_struct;
	gpio_default_para_init(&gpio_init_struct);
	/* configure PB1 tmr3_ch4 as output*/
	gpio_init_struct.gpio_pins = GPIO_PINS_1;
	gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
	gpio_init(GPIOB, &gpio_init_struct);

	gpio_init_struct.gpio_pins = GPIO_PINS_13;
	gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
	//gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
	gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_init(GPIOC, &gpio_init_struct);
}

/* init tmr1 function. */
void wk_tmr1_init(void) {
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
	timer_period = (crm_clocks_freq_struct.sclk_freq / 20000) - 1;

	//  /* compute ccr1 value to generate a duty cycle at 50% for channel 1 and 1n */
	channel1_pulse = (uint16_t)(((uint32_t) 5 * (timer_period - 1)) / 10);

	/* compute ccr2 value to generate a duty cycle at 37.5%  for channel 2 and 2n */
	channel2_pulse = (uint16_t) (((uint32_t) 375 * (timer_period - 1)) / 1000);

	/* compute ccr3 value to generate a duty cycle at 25%  for channel 3 and 3n */
	channel3_pulse = (uint16_t) (((uint32_t) 25 * (timer_period - 1)) / 100);

	/* compute ccr4 value to generate a duty cycle at 12.5%  for channel 4 */
	channel4_pulse = (uint16_t)(((uint32_t) 125 * (timer_period- 1)) / 1000);
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

	/* channel 1 */
	tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_1, &tmr_output_struct);
	tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, channel1_pulse);

	/* channel 2 */
	tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_2, &tmr_output_struct);
	tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, channel2_pulse);

	/* channel 3 */
	tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_3, &tmr_output_struct);
	tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, channel3_pulse);

	/* channel 4 */
	tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_4, &tmr_output_struct);
	tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_4, channel4_pulse);

	/* output enable */
	tmr_output_enable(TMR1, TRUE);

	/* enable tmr1 */
	tmr_counter_enable(TMR1, TRUE);
}

void dma_configuration(void)
{
	dma_init_type dma_init_struct = {0};
	uint16_t index = 0;

	while(index < PWM_DataLength)
	{
		src_buffer[index] = (uint16_t) (((uint32_t) (src_buffer[index]) * (TMRx_PR)) / (1000));
		index++;
	}

  /* dma1 channel7 configuration */
  dma_default_para_init(&dma_init_struct);

  dma_init_struct.buffer_size = PWM_DataLength;
  dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
  dma_init_struct.memory_base_addr = (uint32_t)src_buffer;
  dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
  dma_init_struct.memory_inc_enable = TRUE;
  dma_init_struct.peripheral_base_addr = (uint32_t)&TMR3->c4dt;
  dma_init_struct.peripheral_data_width= DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
  dma_init_struct.peripheral_inc_enable  = FALSE;
  dma_init_struct.priority = DMA_PRIORITY_HIGH;
  dma_init_struct.loop_mode_enable = FALSE;
  dma_init(DMA1_CHANNEL3, &dma_init_struct);
}


void tmr3_configuration(void)
{
  /* Init TMR3 */
	//2603879
  tmr_base_init(TMR3, TMRx_PR-1, 4798);
  tmr_cnt_dir_set(TMR3, TMR_COUNT_UP);

	/* TMR configuration as output mode */
  tmr_output_default_para_init(&tmr_output_struct);
  tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  tmr_output_struct.oc_output_state = TRUE;
  tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_LOW;

	/* TMR3 channel 4 configuration */
  tmr_output_channel_config(TMR3, TMR_SELECT_CHANNEL_4, &tmr_output_struct);

	/* enable tmr output channel buffer */
  tmr_output_channel_buffer_enable(TMR3, TMR_SELECT_CHANNEL_4, TRUE);
}


void wk_exint_config(void)
{
  /* add user code begin exint_config 0 */

  /* add user code end exint_config 0 */

  gpio_init_type gpio_init_struct;
  exint_init_type exint_init_struct;

  /* add user code begin exint_config 1 */

  /* add user code end exint_config 1 */

  /* configure the EXINT12 */
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pins = GPIO_PINS_0;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOC, &gpio_init_struct);

  gpio_exint_line_config(GPIO_PORT_SOURCE_GPIOC, GPIO_PINS_SOURCE0);

  exint_default_para_init(&exint_init_struct);
  exint_init_struct.line_enable = TRUE;
  exint_init_struct.line_mode = EXINT_LINE_INTERRUPUT;
  exint_init_struct.line_select = EXINT_LINE_0;
  exint_init_struct.line_polarity = EXINT_TRIGGER_RISING_EDGE;
  exint_init(&exint_init_struct);

  /**
   * Users need to configure EXINT12 interrupt functions according to the actual application.
   * 1. Call the below function to enable the corresponding EXINT12 interrupt.
   *     --exint_interrupt_enable(EXINT_LINE_12, TRUE);
   * 2. Add the user's interrupt handler code into the below function in the at32f403a_407_int.c file.
   *     --void EXINT15_10_IRQHandler(void)
   */

  /* add user code begin exint_config 2 */

  /* add user code end exint_config 2 */
}


