/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
#include "main.h"
#include "stm32f4xx_hal.h"
#include "dma.h"
//#include "fatfs.h"
#include "sdio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "pid.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint8_t _1ms();
uint8_t _10ms();
uint8_t _50ms();
uint8_t _100ms();
uint8_t _1s();
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

uint8_t flag_1ms=0,flag_10ms=0,flag_50ms=0,flag_100ms=0,flag_1s=0;
uint8_t LOCK=1; //是否解锁飞行器
uint8_t sensor_data[11];   //传感器接收数组
uint8_t control_data[32];  //遥控器接收数组
uint16_t tim2_base=0;    //tim2计数变量
uint16_t control_speed=350; //来自遥控器的油门
uint16_t last_speed=1000;
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SDIO_SD_Init();
  //MX_FATFS_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
	
  /* USER CODE BEGIN 2 */
	//外设初始化
	HAL_GPIO_WritePin(GPIOA,LED4_Pin,GPIO_PIN_SET);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_UART_Receive_DMA(&huart2,sensor_data,11);
	HAL_GPIO_WritePin(GPIOA,LED7_Pin,GPIO_PIN_SET);
	NRF24L01_Init();
	while(NRF24L01_Check());
	NRF24L01_RX_Mode();
	HAL_GPIO_WritePin(GPIOA,LED7_Pin,GPIO_PIN_RESET);
	TIM4_PWM_Open();
	//关闭电机
	set_motor_speed(1000,1000,1000,1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
		
  /* USER CODE BEGIN 3 */
		//printf("%d, %d, %d, %d, %d, %d, %d, %d\r\n",control_data[0],control_data[1],control_data[2],control_data[3],control_data[4],control_data[5],control_data[6],control_data[7]);
		//printf("pitch: %.3f   row: %.3f   yao: %.3f\r\n",sensor_value.ang_p,sensor_value.ang_r,sensor_value.ang_y);
		//printf("accx: %.3f   accy: %.3f   accz: %.3f\r\n",sensor_value.acc_x,sensor_value.acc_y,sensor_value.acc_z);
		
		pid_value.row=0;
		pid_value.pitch=0;
		pid_value.core_d=9.5;
		pid_value.core_i=0.012;
		pid_value.core_p=1.191;
		pid_value.shell_d=0;
		pid_value.shell_i=0;
		pid_value.shell_p=0;
		pid_value.yaw=0;
		
		//1ms周期任务
		if(_1ms())
		{
			
		}
		//10ms周期任务
		if(_10ms())
		{
			//pid
			if(!LOCK)
				PID_Control();
			else
				set_motor_speed(1000,1000,1000,1000);
		}
		//50ms周期任务
		if(_50ms())
		{
			if(NRF24L01_RxPacket(control_data)==0)
			{
				control_data_analysis(control_data);
				//set_motor_speed(BASE_SPEED+control_speed,BASE_SPEED+control_speed,BASE_SPEED+control_speed,BASE_SPEED+control_speed);
				HAL_GPIO_WritePin(GPIOA,LED6_Pin,GPIO_PIN_SET);
				//printf("speed: %d",control_speed);
			}
			else 
			{
				HAL_GPIO_WritePin(GPIOA,LED6_Pin,GPIO_PIN_RESET);
				//printf("no recive\r\n");
			}
		}
		//100ms周期任务
		if(_100ms())
		{
			
		}
		//1s周期任务
		if(_1s())
		{
			
		}
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
//串口回调
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART2)
	{
		HAL_UART_Receive_DMA(huart,sensor_data,11);
		copy_IMU_data(sensor_data);
	}
}

//定时器回调
uint16_t _1ms_count=1,_10us_count=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM1)
	{
		
	}
	else if(htim->Instance==TIM2)    //周期任务计时器
	{
		tim2_base++;
		if((tim2_base%1)==0) 
			flag_1ms=1;
		if((tim2_base%10)==0) 
			flag_10ms=1;
		if((tim2_base%50)==0) 
			flag_50ms=1;
		if((tim2_base%100)==0) 
			flag_100ms=1;
		if(tim2_base==1000)
		{
			flag_1s=1;
			tim2_base=0;
		}
	}
	else if(htim->Instance==TIM3)    //系统灯计时器
	{
		if(_1ms_count==1) {HAL_GPIO_WritePin(GPIOA,LED4_Pin,GPIO_PIN_SET);_1ms_count++;}
		else if(_1ms_count==70) {HAL_GPIO_WritePin(GPIOA,LED4_Pin,GPIO_PIN_RESET);_1ms_count++;}
		else if(_1ms_count==1000) _1ms_count=1;
		else _1ms_count++;
	}
}

uint8_t _1ms()
{
	if(flag_1ms)
	{
		flag_1ms=0;
		return 1;
	}
	else
		return 0;
}
uint8_t _10ms()
{
	if(flag_10ms)
	{
		flag_10ms=0;
		return 1;
	}
	else
		return 0;
}
uint8_t _50ms()
{
	if(flag_50ms)
	{
		flag_50ms=0;
		return 1;
	}
	else
		return 0;
}
uint8_t _100ms()
{
	if(flag_100ms)
	{
		flag_100ms=0;
		return 1;
	}
	else
		return 0;
}
uint8_t _1s()
{
	if(flag_1s)
	{
		flag_1s=0;
		return 1;
	}
	else
		return 0;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
