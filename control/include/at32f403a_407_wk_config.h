/*
 * at32f403a_407_wk_config.h
 *
 *  Created on: 2024 Jun 6
 *      Author: user
 */
#include "at32f403a_407.h"

#define TMRx_PR 1000
#define BIT_NUMBER 8

#define I2C_TIMEOUT                      0xffffffFF

#define I2Cx_SPEED                       100000
#define I2Cx_ADDRESS                     0x15<<1

#define I2Cx_PORT                        I2C2
#define I2Cx_CLK                         CRM_I2C2_PERIPH_CLOCK

#define I2Cx_SCL_PIN                     GPIO_PINS_10
#define I2Cx_SCL_GPIO_PORT               GPIOB
#define I2Cx_SCL_GPIO_CLK                CRM_GPIOB_PERIPH_CLOCK

#define I2Cx_SDA_PIN                     GPIO_PINS_11
#define I2Cx_SDA_GPIO_PORT               GPIOB
#define I2Cx_SDA_GPIO_CLK                CRM_GPIOB_PERIPH_CLOCK

#define I2Cx_EVT_IRQn                    I2C2_EVT_IRQn
#define I2Cx_ERR_IRQn                    I2C2_ERR_IRQn


#ifndef INCLUDE_AT32F403A_407_WK_CONFIG_H_
#define INCLUDE_AT32F403A_407_WK_CONFIG_H_
/* includes -----------------------------------------------------------------------*/
extern gpio_init_type  gpio_init_struct;
extern tmr_output_config_type tmr_output_struct;
extern crm_clocks_freq_type crm_clocks_freq_struct;
extern uint16_t timer_period;
extern uint16_t PWM_DataLength;
extern uint16_t channel1_pulse, channel2_pulse, channel3_pulse, channel4_pulse;
extern uint16_t src_buffer[];
/* private includes -------------------------------------------------------------*/
/* add user code begin private includes */

/* add user code end private includes */

/* exported types -------------------------------------------------------------*/
/* add user code begin exported types */

/* add user code end exported types */

/* exported constants --------------------------------------------------------*/
/* add user code begin exported constants */

/* add user code end exported constants */

/* exported macro ------------------------------------------------------------*/
/* add user code begin exported macro */

/* add user code end exported macro */

/* exported functions ------------------------------------------------------- */
  /* system clock config. */
  void wk_system_clock_config(void);

  /* config periph clock. */
  void wk_periph_clock_config(void);

  /* nvic config. */
  void wk_nvic_config(void);

  /* init tmr6 function. */
  void wk_tmr6_init(void);

  /* init crm function. */
  void crm_init(void);

  void wk_gpio_init(void);

  /* init tmr1 function. */
  void wk_tmr1_init(void);

  /* init exint function. */
  void wk_exint_config(void);

  void dma_configuration(void);

  void tmr3_configuration(void);

/* add user code begin exported functions */

/* add user code end exported functions */


#endif /* INCLUDE_AT32F403A_407_WK_CONFIG_H_ */
