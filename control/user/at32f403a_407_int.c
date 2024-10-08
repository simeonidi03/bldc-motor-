/**
  **************************************************************************
  * @file     at32f403a_407_int.c
  * @version  v2.1.2
  * @date     2022-08-16
  * @brief    main interrupt service routines.
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */

/* includes ------------------------------------------------------------------*/
#include "at32f403a_407_int.h"
#include "i2c_application.h"

extern i2c_handle_type hi2cx;

#define I2Cx_EVT_IRQHandler              I2C2_EVT_IRQHandler
#define I2Cx_ERR_IRQHandler              I2C2_ERR_IRQHandler

extern void usart2_tx_rx_handler(void);



/** @addtogroup AT32F403A_periph_examples
  * @{
  */

/** @addtogroup 403A_TMR_7_pwm_output
  * @
  */

/**
  * @brief  this function handles nmi exception.
  * @param  none
  * @retval none
  */
void NMI_Handler(void)
{
}

/**
  * @brief  this function handles hard fault exception.
  * @param  none
  * @retval none
  */
void HardFault_Handler(void)
{
  /* go to infinite loop when hard fault exception occurs */
  while(1)
  {
  }
}

/**
  * @brief  this function handles memory manage exception.
  * @param  none
  * @retval none
  */
void MemManage_Handler(void)
{
  /* go to infinite loop when memory manage exception occurs */
  while(1)
  {
  }
}

/**
  * @brief  this function handles bus fault exception.
  * @param  none
  * @retval none
  */
void BusFault_Handler(void)
{
  /* go to infinite loop when bus fault exception occurs */
  while(1)
  {
  }
}

/**
  * @brief  this function handles usage fault exception.
  * @param  none
  * @retval none
  */
void UsageFault_Handler(void)
{
  /* go to infinite loop when usage fault exception occurs */
  while(1)
  {
  }
}

/**
  * @brief  this function handles svcall exception.
  * @param  none
  * @retval none
  */
void SVC_Handler(void)
{
}

/**
  * @brief  this function handles debug monitor exception.
  * @param  none
  * @retval none
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  this function handles pendsv_handler exception.
  * @param  none
  * @retval none
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  this function handles systick handler.
  * @param  none
  * @retval none
  */
void SysTick_Handler(void)
{
  /* trigger TMR1 hall event */
  tmr_event_sw_trigger(TMR1, TMR_HALL_SWTRIG);
}


//		if (direction == 0) {
//			odometr_div18++;
//		} else {
//			odometr_div18--;
//		}

void EXINT15_10_IRQHandler(void)
{
	// Проверка, что прерывание вызвано именно линией 13
	if (EXINT->intsts & EXINT_LINE_13) {
		OdometrProcess(motorA_ptr);
		EXINT->intsts = EXINT_LINE_13;
	}
}

void EXINT0_IRQHandler(void)
{
	if (EXINT->intsts & EXINT_LINE_0) {
		OdometrProcess(motorB_ptr);
		EXINT->intsts = EXINT_LINE_0;
	}
}




void USART2_IRQHandler(void)
{
  usart2_tx_rx_handler();
}

void TMR6_GLOBAL_IRQHandler(void)
{
  TMR6->ists_bit.ovfif = 0;
//  CalcParrot(motorA_ptr);
  CalcPid(motorA_ptr);
//  CalcPid(motorB_ptr);
  //usart2_tx_without_int();

  //odometr_next = odometr_div18;
  //speed_hall = (odometr_next - odometr_bef)/2;
  //odometr_bef = odometr_div18;

  /* add user code begin TMR6_GLOBAL_IRQ 0 */

  /* add user code end TMR6_GLOBAL_IRQ 0 */
  /* add user code begin TMR6_GLOBAL_IRQ 1 */

  /* add user code end TMR6_GLOBAL_IRQ 1 */
}

/**
  * @brief  this function handles usart3 handler.
  * @param  none
  * @retval none
  */
void USART3_IRQHandler(void)
{

}



/**
  * @brief  this function handles i2c error interrupt request.
  * @param  none
  * @retval none
  */

