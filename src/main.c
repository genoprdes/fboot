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
 * @file main.c
 * @author Shenzhen Feibit Electronic Technology Co.,Ltd.
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2021,FeiBit Technologies Inc. All rights reserved.
 */
#include "main.h"
#include "FZX6180MT.h"
#include "log.h"
#include "bootloader.h"
#include "hal_timer.h"
#include "6180mt_common.h"
#include "hal_gpio.h"
#include <stdio.h>
#include "hal_aes.h"

#include "ble_stack_common.h"
#include "app.h"
#include "app_rdtss.h"
/** @addtogroup FZX6180MT_StdPeriph_Examples
 * @{
 */

/** @addtogroup DMA_SPI_RAM
 * @{
 */

extern uint8_t BLSP_Boot2_Image_Valid_Check(void);
void RCC_Configuration(void);
void GPIO_Configuration(void);
void DMA_ParameterConfiguration(void);
void SPI_ParameterConfiguration(void);
void LED_Init(void);
void TIM6_NVIC_Configuration(void);
void SPI_DMA_Init(void);
void UART_DMA_Init(void);
unsigned char Uint8ToChar(int8_t hexChar);

extern uint32_t xz_start_addr;
extern uint32_t xz_prom_start_addr;
uint8_t SPI_Master_Tx_Buffer[BUFFER_SIZE] ;//= {0x01, 0xA0};
uint8_t SPI_Master_Rx_Buffer[BUFFER_SIZE];
uint8_t UART_Rx_Buffer[UART_BUFFER_SIZE];
struct status_flg Status;

extern Fbee_BootMap fbee_bootAddress;
//extern void CRC16TableCalculate(void);
//extern unsigned short crc16_flash(uint32_t addr, int len);
extern unsigned short crc16_ccitt(const unsigned char *buf, int len);
/**
 * @brief  Main program
 */
uint8_t ieee_addr[8];
//#define TEST_ADDR              	 0X1008000
int main(void)
{
    int alienmsg_bl_mode;
//    CRC16TableCalculate();
    log_init();
    SPI_DMA_Init();
	UART_DMA_Init();	
    USART_EnableDMA(USART1, USART_DMAREQ_RX, DISABLE);/* DISABLE USAR1 DMA Rx request */
    DMA_EnableChannel(DMA_CH1, DISABLE);/* DISABLE USAR1 RX DMA Channel */
    Qflash_Init();/*Initialize Qflash.*/
    log_s(BLVERSIONSTRING" is running@0x01000000"NONE);
    log_s("COMPILE TIME: [ %s %s ]\r\n", __DATE__, __TIME__);
	
    BLSP_Boot2_Image_Valid_Check();
//	log("CRC = 0x%X\r\n",crc16_flash(FBOOT_APP0_BASE_START, 1120));
//	log("crc = 0x%X\r\n",crc16_ccitt((unsigned char*)FBOOT_APP0_BASE_START, 1120 * 128));

    alienmsg_bl_mode = Check_APP_FLG();
    LED_Init();
//    /* TIM6 NVIC Configuration */
//    TIM6_NVIC_Configuration();
//    /* TIM6 Configuration */
//    TIM6_Configuration();
//    /* TIM6 disable counter */
//    TIM_Enable(TIM6, DISABLE);
    log("fullcode_start = 0x%X,fullcode_end = 0x%X \r\n", fbee_bootAddress.fullcode_start ,fbee_bootAddress.fullcode_end);
    if (Status.BLMode != OADMODE)
    {
        Status.BLMode = INVALID_DATA;
        Qflash_Erase_Write(FBOOT_STATUS_PAGE_START, (uint8_t *)&Status, sizeof(Status));
        /*重新写一次app起始和结束地址，因为在上一行把标志擦除了*/
        Qflash_Write(FBOOT_STATUS_BOOTMAP, (uint8_t *)&fbee_bootAddress, sizeof(fbee_bootAddress));
    }
    /* TIM6 enable counter */
//    TIM_Enable(TIM6, ENABLE);/*TIM影响BLE*/

	Qflash_Read(FBOOT_INFO_BASE_START, (uint8_t *)(ieee_addr), 8);
	
    Switch_BL_mode(alienmsg_bl_mode);
    log_s("main init end \r\n");
    for (;;);
}
/*将输入的十六进制字符转换为对应十六进制数字*/
unsigned char Uint8ToChar(int8_t hexChar)
{
    if (0 <= hexChar && hexChar <= 9)
    {
        return hexChar + '0';
    }
    else if (0x0A <= hexChar && hexChar <= 0x0F)
    {
        return hexChar + 'A' - 10;
    }
    else
    {
        return 0;
    }
}

extern unsigned char BLE_DMA_rx_buff[1036];
void UART_DMA_Init(void)
{
	DMA_InitType DMA_InitStructure;

    /* USARTy TX DMA Channel (triggered by USARTy Tx event) Config */
    DMA_DeInit(DMA_CH1);
    DMA_InitStructure.PeriphAddr     = (USART1_BASE + 0x04);
    DMA_InitStructure.MemAddr        = (uint32_t)BLE_DMA_rx_buff;
    DMA_InitStructure.Direction      = DMA_DIR_PERIPH_SRC;
    DMA_InitStructure.BufSize        = 1030;
    DMA_InitStructure.PeriphInc      = DMA_PERIPH_INC_DISABLE;
    DMA_InitStructure.DMA_MemoryInc  = DMA_MEM_INC_ENABLE;
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
    DMA_InitStructure.MemDataSize    = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.CircularMode   = DMA_MODE_NORMAL;
    DMA_InitStructure.Priority       = DMA_PRIORITY_VERY_HIGH;
    DMA_InitStructure.Mem2Mem        = DMA_M2M_DISABLE;
    DMA_Init(DMA_CH1, &DMA_InitStructure);
    DMA_RequestRemap(DMA_REMAP_USART1_RX, DMA, DMA_CH1, ENABLE);
}
void SPI_DMA_Init(void)
{
    /* System Clocks Configuration */
    RCC_Configuration();
    /* Configure the GPIO ports */
    GPIO_Configuration();
    /*将寄存器恢复为默认值*/
    SPI_I2S_DeInit(SPI_MASTER);
    /*SPI参数配置及初始化*/
    SPI_ParameterConfiguration();
    /*DMA参数配置及初始化*/
    DMA_ParameterConfiguration();
    /* Disable SPI_MASTER DMA Tx request */
    SPI_I2S_EnableDma(SPI_MASTER, SPI_I2S_DMA_TX, DISABLE);
    /* Disable SPI_MASTER DMA Rx request */
    SPI_I2S_EnableDma(SPI_MASTER, SPI_I2S_DMA_RX, DISABLE);
    /* Disable SPI_MASTER CRC calculation */
    SPI_EnableCalculateCrc(SPI_MASTER, DISABLE);
    /* Enable SPI_MASTER */
    SPI_Enable(SPI_MASTER, ENABLE);
}
/**
 * @brief  Configures the different system clocks.
 */
void RCC_Configuration(void)
{
    /* Enable peripheral clocks
     * ------------------------------------------------*/
    /* PCLK1 = HCLK/4 */
    RCC_ConfigPclk1(RCC_HCLK_DIV4);

    /* TIM6 clock enable */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM6, ENABLE);

    /* Enable DMA clock */
    RCC_EnableAHBPeriphClk(SPI_MASTER_DMA_CLK, ENABLE);

    /* Enable SPI_MASTER clock and GPIO clock for SPI_MASTER */
    RCC_EnableAPB2PeriphClk(SPI_MASTER_GPIO_CLK | SPI_MASTER_CLK | RCC_APB2_PERIPH_AFIO, ENABLE);
}

/**
 * @brief  Configures the different GPIO ports.
 */
void GPIO_Configuration(void)
{

    GPIO_InitType GPIO_InitStructure;

    GPIO_InitStruct(&GPIO_InitStructure);
    /* Configure SPI_MASTER pins: SCK 、MISO and MOSI */
    GPIO_InitStructure.Pin        = SPI_MASTER_PIN_SCK | SPI_MASTER_PIN_MOSI | SPI_MASTER_PIN_MISO;
    GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_HIGH;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Pull  = GPIO_NO_PULL;
    GPIO_InitStructure.GPIO_Alternate = SPI_MASTER_GPIO_AF;
    GPIO_InitPeripheral(SPI_MASTER_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.Pin       = SPI_MASTER_PIN_CS;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitPeripheral(SPI_MASTER_GPIO, &GPIO_InitStructure);
    /*把CS拉高*/
    GPIO_SetBits(SPI_MASTER_GPIO, SPI_MASTER_PIN_CS);
}

void SPI_ParameterConfiguration(void)
{
    SPI_InitType SPI_InitStructure;

    /* SPI_MASTER configuration
     * ------------------------------------------------*/
    SPI_InitStructure.DataDirection = SPI_DIR_DOUBLELINE_FULLDUPLEX;
    SPI_InitStructure.SpiMode       = SPI_MODE_MASTER;
    SPI_InitStructure.DataLen       = SPI_DATA_SIZE_8BITS;
    SPI_InitStructure.CLKPOL        = SPI_CLKPOL_LOW;
    SPI_InitStructure.CLKPHA        = SPI_CLKPHA_FIRST_EDGE;
    SPI_InitStructure.NSS           = SPI_NSS_SOFT;
    SPI_InitStructure.BaudRatePres  = SPI_BR_PRESCALER_16;
    SPI_InitStructure.FirstBit      = SPI_FB_MSB;
//    SPI_InitStructure.CRCPoly       = CRC_POLYNOMIAL;
    SPI_Init(SPI_MASTER, &SPI_InitStructure);
}

void DMA_ParameterConfiguration(void)
{
    DMA_InitType DMA_InitStructure;

    /* SPI_MASTER_RX_DMA_CHANNEL configuration
     * ---------------------------------*/
    DMA_DeInit(SPI_MASTER_RX_DMA_CHANNEL);

    DMA_InitStructure.PeriphAddr     = (uint32_t)&SPI1->DAT;
    DMA_InitStructure.MemAddr        = (uint32_t)SPI_Master_Rx_Buffer;
    DMA_InitStructure.Direction      = DMA_DIR_PERIPH_SRC;
    DMA_InitStructure.BufSize        = BUFFER_SIZE;
    DMA_InitStructure.PeriphInc      = DMA_PERIPH_INC_DISABLE;
    DMA_InitStructure.DMA_MemoryInc  = DMA_MEM_INC_ENABLE;
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
    DMA_InitStructure.MemDataSize    = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.CircularMode   = DMA_MODE_NORMAL;
    DMA_InitStructure.Priority       = DMA_PRIORITY_HIGH;
    DMA_InitStructure.Mem2Mem        = DMA_M2M_DISABLE;
    DMA_Init(SPI_MASTER_RX_DMA_CHANNEL, &DMA_InitStructure);
    DMA_RequestRemap(SPI_MASTER_RX_DMA_REMAP, DMA, SPI_MASTER_RX_DMA_CHANNEL, ENABLE);

    /* SPI_MASTER_TX_DMA_CHANNEL configuration
     * ---------------------------------*/
    DMA_DeInit(SPI_MASTER_TX_DMA_CHANNEL);
    DMA_InitStructure.PeriphAddr = (uint32_t)&SPI1->DAT;
    DMA_InitStructure.MemAddr    = (uint32_t)SPI_Master_Tx_Buffer;
    DMA_InitStructure.Direction  = DMA_DIR_PERIPH_DST;
    DMA_InitStructure.Priority   = DMA_PRIORITY_LOW;
    DMA_Init(SPI_MASTER_TX_DMA_CHANNEL, &DMA_InitStructure);
    DMA_RequestRemap(SPI_MASTER_TX_DMA_REMAP, DMA, SPI_MASTER_TX_DMA_CHANNEL, ENABLE);

    /* USART1 TX DMA Channel Config */
//    DMA_DeInit(LOG_USARx_DMA);
//    DMA_InitStructure.PeriphAddr     = USART1_BASE + 0x04;
//    DMA_InitStructure.MemAddr        = (uint32_t)UART_Rx_Buffer;
//    DMA_InitStructure.Direction      = DMA_DIR_PERIPH_SRC;
//    DMA_InitStructure.BufSize        = UART_BUFFER_SIZE;
//    DMA_InitStructure.PeriphInc      = DMA_PERIPH_INC_DISABLE;
//    DMA_InitStructure.DMA_MemoryInc  = DMA_MEM_INC_ENABLE;
//    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
//    DMA_InitStructure.MemDataSize    = DMA_MemoryDataSize_Byte;
//    DMA_InitStructure.CircularMode   = DMA_MODE_NORMAL;
//    DMA_InitStructure.Priority       = DMA_PRIORITY_VERY_HIGH;
//    DMA_InitStructure.Mem2Mem        = DMA_M2M_DISABLE;
//    DMA_Init(LOG_USARx_DMA, &DMA_InitStructure);
//    DMA_RequestRemap(DMA_REMAP_USART1_RX, DMA, LOG_USARx_DMA, ENABLE);

}

/**
 * @}
 */
void LED_Init()
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_InitStruct(&GPIO_InitStructure);
	
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);
    GPIO_InitStructure.Pin = LED_R|LED_G|LED_B|LED_W;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_HIGH;
    GPIO_InitPeripheral(LED_GPIO, &GPIO_InitStructure);
}
void All_EnableIRQ()
{
    GPIO_INTERUPT_DIS_EN(TRUE);/*开启GPIO中断*/
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_EnableIRQ(TIM6_IRQn);
    NVIC_EnableIRQ(TIM3_IRQn);
}

void All_DisableIRQ()
{
    GPIO_INTERUPT_DIS_EN(FALSE);/*关闭GPIO中断*/
    NVIC_DisableIRQ(USART1_IRQn);
    NVIC_DisableIRQ(TIM6_IRQn);
    NVIC_DisableIRQ(TIM3_IRQn);
}
/**
 * @}
 */
