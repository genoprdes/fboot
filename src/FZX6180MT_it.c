/*****************************************************************************
 * Copyright (c) 2021,FeiBit Technologies Inc.
 *
 * All rights reserved.
 * ****************************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * FitBit name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY FEIBIT "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL FEIBIT BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ****************************************************************************/

/**
 * @file n32g43x_it.c
 * @author Shenzhen Feibit Electronic Technology Co.,Ltd.
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2021,FeiBit Technologies Inc. All rights reserved.
 */
#include "FZX6180MT_it.h"
#include "FZX6180MT.h"
#include "FZX6180MT_gpio.h"
#include "log.h"

extern unsigned long demotimer0count;

/** @addtogroup N32G43X_StdPeriph_Template
 * @{
 */

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
 * @brief  This function handles NMI exception.
 */
void NMI_Handler(void)
{
}

/**
 * @brief  This function handles Hard Fault exception.
 */
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
	#if 0
    while (1)
    {
    }
	#else
	log_s("HardFault\r\n");
	NVIC_SystemReset();
	#endif
}

/**
 * @brief  This function handles SVCall exception.
 */
void SVC_Handler(void)
{
}

/**
 * @brief  This function handles PendSV_Handler exception.
 */
void PendSV_Handler(void)
{
}

/**
 * @brief  This function handles SysTick Handler.
 */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 N32G43X Peripherals Interrupt Handlers                     */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_n32g43x.s).                                                 */
/******************************************************************************/
/**
 * @brief  This function handles TIM6 global interrupt request.
 */
void TIM6_IRQHandler(void)
{
    if (TIM_GetIntStatus(TIM6, TIM_INT_UPDATE) != RESET)
    {
        TIM_ClrIntPendingBit(TIM6, TIM_INT_UPDATE);
		demotimer0count++; 
		
		GPIO_WriteBit(GPIOA,GPIO_PIN_6,(Bit_OperateType)(!GPIO_ReadOutputDataBit(GPIOA, GPIO_PIN_6)));
		GPIO_WriteBit(GPIOB,GPIO_PIN_0,(Bit_OperateType)(!GPIO_ReadOutputDataBit(GPIOB, GPIO_PIN_0)));		
    }
}

/**
 * @brief  This function handles PPP interrupt request.
 */
/*void PPP_IRQHandler(void)
{
}*/

/**
 * @}
 */
