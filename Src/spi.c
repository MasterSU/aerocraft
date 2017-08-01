/**
  ******************************************************************************
  * File Name          : SPI.c
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

/* Includes ------------------------------------------------------------------*/
#include "spi.h"

#include "main.h"
#include "gpio.h"
#include "pid.h"

/* USER CODE BEGIN 0 */
const uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; 
const uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; 
/* USER CODE END 0 */

SPI_HandleTypeDef hspi2;

/* SPI2 init function */
void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspInit 0 */

  /* USER CODE END SPI2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();
  
    /**SPI2 GPIO Configuration    
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_MspInit 1 */

  /* USER CODE END SPI2_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();
  
    /**SPI2 GPIO Configuration    
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);

  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
  }
} 


void SPI2_SetSpeed(uint8_t SPI_BaudRatePrescaler)
{
	assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
	__HAL_SPI_DISABLE(&hspi2);           
	hspi2.Instance->CR1&=0XFFC7;        
	hspi2.Instance->CR1|=SPI_BaudRatePrescaler;
	__HAL_SPI_ENABLE(&hspi2);            
    
}

uint8_t SPI2_ReadWriteByte(uint8_t TxData)
{
	uint8_t Rxdata;
	HAL_SPI_TransmitReceive(&hspi2,&TxData,&Rxdata,1, 1000);       
	return Rxdata;          		 
}

void NRF24L01_CE(uint8_t dat)
{
	if(dat) HAL_GPIO_WritePin(CE_GPIO_Port,CE_Pin,GPIO_PIN_SET);
	else HAL_GPIO_WritePin(CE_GPIO_Port,CE_Pin,GPIO_PIN_RESET);
}

void NRF24L01_CSN(uint8_t dat)
{
	if(dat) HAL_GPIO_WritePin(CSN_GPIO_Port,CSN_Pin,GPIO_PIN_SET);
	else HAL_GPIO_WritePin(CSN_GPIO_Port,CSN_Pin,GPIO_PIN_RESET);
}

uint8_t NRF24L01_IRQ()
{
	if(HAL_GPIO_ReadPin(IRQ_GPIO_Port,IRQ_Pin)==GPIO_PIN_SET) return 1;
	else return 0;
}
 

void NRF24L01_Init(void)
{  
	NRF24L01_CE(0); 			
	NRF24L01_CSN(1);			 	 
}

uint8_t NRF24L01_Check(void)
{
	uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	uint8_t i;
	SPI2_SetSpeed(SPI_BAUDRATEPRESCALER_8);   	 
	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,buf,5);
	NRF24L01_Read_Buf(TX_ADDR,buf,5);   
	for(i=0;i<5;i++)if(buf[i]!=0XA5)break;	 							   
	if(i!=5)return 1;
	return 0;		
}	 	 

uint8_t NRF24L01_Write_Reg(uint8_t reg,uint8_t value)
{
	uint8_t status;	
  NRF24L01_CSN(0);               
  status =SPI2_ReadWriteByte(reg);
  SPI2_ReadWriteByte(value);      
  NRF24L01_CSN(1);                   
  return(status);       		
}

uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;	    
  NRF24L01_CSN(0);        
  SPI2_ReadWriteByte(reg);  
  reg_val=SPI2_ReadWriteByte(0XFF);
  NRF24L01_CSN(1);        
  return(reg_val);         
}	

uint8_t NRF24L01_Read_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
  uint8_t status,uint8_t_ctr;	       
  NRF24L01_CSN(0);           
  status=SPI2_ReadWriteByte(reg);	   
  for(uint8_t_ctr=0;uint8_t_ctr<len;uint8_t_ctr++)pBuf[uint8_t_ctr]=SPI2_ReadWriteByte(0XFF);
  NRF24L01_CSN(1);     
  return status;     
}

uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
  uint8_t status,uint8_t_ctr;	    
 	NRF24L01_CSN(0);        
  status = SPI2_ReadWriteByte(reg);
  for(uint8_t_ctr=0; uint8_t_ctr<len; uint8_t_ctr++)SPI2_ReadWriteByte(*pBuf++);
  NRF24L01_CSN(1);   
  return status;     
}				   

uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{
	uint8_t sta;
 	SPI2_SetSpeed(SPI_BAUDRATEPRESCALER_8);
	NRF24L01_CE(0);
  NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);
 	NRF24L01_CE(1);
	while(NRF24L01_IRQ()!=0);
	sta=NRF24L01_Read_Reg(STATUS); 
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); 
	if(sta&MAX_TX)
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);
		return MAX_TX; 
	}
	if(sta&TX_OK)
	{
		return TX_OK;
	}
	return 0xff;
}

uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
	uint8_t sta;		    							   
	SPI2_SetSpeed(SPI_BAUDRATEPRESCALER_8); 
	sta=NRF24L01_Read_Reg(STATUS); 
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta);
	if(sta&RX_OK)
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);
		NRF24L01_Write_Reg(FLUSH_RX,0xff);
		return 0; 
	}	   
	return 1;
}					    
  
void NRF24L01_RX_Mode(void)
{
  NRF24L01_CE(0);	  
  NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);
	  
  NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01); 
  NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);
  NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,40);	    
  NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);
  NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);
  NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG, 0x0f);
  NRF24L01_CE(1); 
}						 

void NRF24L01_TX_Mode(void)
{														 
	NRF24L01_CE(0);	    
  NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);
  NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); 	  

  NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);     
  NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);
  NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);
  NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,40);       
  NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f); 
  NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG,0x0e);  
	NRF24L01_CE(1);
}

void control_data_analysis(uint8_t *data)
{
	uint8_t i=0;
	for(i=0;i<RX_PLOAD_WIDTH;i++)
	{
		if(data[i])
		{
			switch(i)
			{
				case ym_up:
						control_speed+=20;
						if(control_speed>500) control_speed=500;
					break;
				case ym_down:
						if(control_speed==0) break;
						else control_speed-=20;
					break;
				case p_up:
						pid_value.core_p+=0.1;
					break;
				case p_down:
						pid_value.core_p-=0.1;
						if(pid_value.core_p<0) pid_value.core_p=0;
					break;
				case d_up:
						pid_value.core_d+=0.1;
					break;
				case d_down:
					pid_value.core_d-=0.1;
					if(pid_value.core_d<0) pid_value.core_d=0;
					break;
				case start:
						LOCK=0;
					break;
				case stop:
						LOCK=1;
					break;
			}
		}
	}
}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
