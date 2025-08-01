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
 * @file main.h
 * @author Shenzhen Feibit Electronic Technology Co.,Ltd.
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2021,FeiBit Technologies Inc. All rights reserved.
 */
#ifndef __MAIN_H__
#define __MAIN_H__

#include "FZX6180MT.h"

#define UART_BUFFER_SIZE    	    32
#define BUFFER_SIZE    			      256
#define SPI_MASTER                SPI1
#define SPI_MASTER_CLK            RCC_APB2_PERIPH_SPI1
#define SPI_MASTER_GPIO           GPIOA
#define SPI_MASTER_GPIO_AF        GPIO_AF1_SPI1
#define SPI_MASTER_GPIO_CLK       RCC_APB2_PERIPH_GPIOA
#define SPI_MASTER_PIN_CS         GPIO_PIN_0
#define SPI_MASTER_PIN_SCK        GPIO_PIN_1
#define SPI_MASTER_PIN_MISO       GPIO_PIN_3
#define SPI_MASTER_PIN_MOSI       GPIO_PIN_2
#define SPI_MASTER_DMA            DMA
#define SPI_MASTER_DMA_CLK        RCC_AHB_PERIPH_DMA
#define SPI_MASTER_RX_DMA_CHANNEL DMA_CH2
#define SPI_MASTER_RX_DMA_FLAG    DMA_FLAG_TC2
#define SPI_MASTER_RX_DMA_REMAP   DMA_REMAP_SPI1_RX
#define SPI_MASTER_TX_DMA_CHANNEL DMA_CH3
#define SPI_MASTER_TX_DMA_FLAG    DMA_FLAG_TC3
#define SPI_MASTER_TX_DMA_REMAP   DMA_REMAP_SPI1_TX

#define LED_GPIO		GPIOB
#define LED_R			GPIO_PIN_2
#define LED_G        	GPIO_PIN_3
#define LED_B       	GPIO_PIN_4
#define LED_W       	GPIO_PIN_5


void All_DisableIRQ(void);
void All_EnableIRQ(void);

#endif /* __MAIN_H__ */
