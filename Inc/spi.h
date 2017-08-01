/**
  ******************************************************************************
  * File Name          : SPI.h
  * Description        : This file provides code for the configuration
  *                      of the SPI instances.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __spi_H
#define __spi_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"

/* USER CODE BEGIN Includes */
#define NRF_READ_REG    0x00 
#define NRF_WRITE_REG   0x20 
#define RD_RX_PLOAD     0x61  
#define WR_TX_PLOAD     0xA0  
#define FLUSH_TX        0xE1  
#define FLUSH_RX        0xE2
#define REUSE_TX_PL     0xE3 
#define NOP             0xFF 
//SPI(NRF24L01)?????
#define CONFIG          0x00  
                              
#define EN_AA           0x01 
#define EN_RXADDR       0x02 
#define SETUP_AW        0x03 
#define SETUP_RETR      0x04  
#define RF_CH           0x05 
#define RF_SETUP        0x06 
#define STATUS          0x07  
                             
#define MAX_TX  		0x10 
#define TX_OK   		0x20 
#define RX_OK   		0x40 

#define OBSERVE_TX      0x08 
#define CD              0x09 
#define RX_ADDR_P0      0x0A  
#define RX_ADDR_P1      0x0B  
#define RX_ADDR_P2      0x0C 
#define RX_ADDR_P3      0x0D 
#define RX_ADDR_P4      0x0E 
#define RX_ADDR_P5      0x0F 
#define TX_ADDR         0x10 
#define RX_PW_P0        0x11 
#define RX_PW_P1        0x12  
#define RX_PW_P2        0x13 
#define RX_PW_P3        0x14 
#define RX_PW_P4        0x15 
#define RX_PW_P5        0x16 
#define NRF_FIFO_STATUS 0x17 
                             

 

#define TX_ADR_WIDTH    5   
#define RX_ADR_WIDTH    5   
#define TX_PLOAD_WIDTH  32  
#define RX_PLOAD_WIDTH  32  	
/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN Private defines */
#define ym_up 0x00
#define ym_down 0x01
#define p_up 0x02
#define p_down 0x03
#define d_up 0x04
#define d_down 0x05
#define start 0x06
#define stop 0x07
/* USER CODE END Private defines */

extern void _Error_Handler(char *, int);

void MX_SPI2_Init(void);

/* USER CODE BEGIN Prototypes */
void NRF24L01_CE(uint8_t dat);
void NRF24L01_CSN(uint8_t dat);
uint8_t NRF24L01_IRQ(void);
void NRF24L01_Init(void);
uint8_t NRF24L01_Check(void);
uint8_t NRF24L01_Write_Reg(uint8_t reg,uint8_t value);
uint8_t NRF24L01_Read_Reg(uint8_t reg);
uint8_t NRF24L01_Read_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len);
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len);
uint8_t NRF24L01_TxPacket(uint8_t *txbuf);
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf);
void NRF24L01_RX_Mode(void);
void NRF24L01_TX_Mode(void);
void control_data_analysis(uint8_t *data);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ spi_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
