/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "i2c_hal.h"
#include "stdio.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KEY1 HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)
#define KEY2 HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)
#define KEY3 HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)
#define KEY4 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//key
u8 key_val,key_down,key_up,key_old;

//lcd
u8 sta_lcd;
u8 ptr[30];

unsigned char tx[21],rx,rx_buf[21];//串口相关变量
u8 dex;
u8 tc[128];
u8 tcs = 0;

//led
u8 led_in;
u8 e2prom[5];//EEPROM存储数组
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void i2c_Read(unsigned char *PucBuff,unsigned char ucAddr,unsigned char ucNum)
{
	I2CStart();
	I2CSendByte(0xa0);
	I2CWaitAck();
	I2CSendByte(ucAddr);
	I2CWaitAck();
	I2CStart();
	I2CSendByte(0xa1);
	I2CWaitAck();
	while(ucNum--)
	{
		*PucBuff++ = I2CReceiveByte();
		if(ucNum)
			I2CSendAck();
		else
			I2CSendNotAck();
	}
	I2CStop();
}

void i2c_Write(unsigned char *PucBuff,unsigned char ucAddr,unsigned char ucNum)
{
	I2CStart();
	I2CSendByte(0xa0);
	I2CWaitAck();
	I2CSendByte(ucAddr);
	I2CWaitAck();
	while(ucNum--)
	{
		I2CSendByte(*PucBuff++);
		I2CWaitAck();
	}
	I2CStop();
	HAL_Delay(5);
}


u8 key_scan()
{
	if((KEY1 == 0)||(KEY2 == 0)||(KEY3 == 0)||(KEY4 == 0))
	{
		HAL_Delay(10);
		if(KEY1 == 0) return 1;
		else if(KEY2 == 0) return 2;
		else if(KEY3 == 0) return 3;
		else if(KEY4 == 0) return 4;
	}return 0;
}

void led_dis(u8 led)
{
	HAL_GPIO_WritePin(GPIOC,0xff00,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
	
	HAL_GPIO_WritePin(GPIOC,led<<8,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
}

void key_Proc()
{
	static __IO uint32_t Key_Tick,DKey_Tick;
	
	if(uwTick - Key_Tick < 50)
		return;
	
	Key_Tick = uwTick;
	
	key_val = key_scan();
	key_down = key_val&(key_val^key_old);
	key_up = ~key_val&(key_val^key_old);
	key_old = key_val;
	
	if(key_down != 0)
		DKey_Tick=uwTick;
	
	if(uwTick - DKey_Tick > 1000)
	{
		switch(key_up)
		{
			case 1:
				led_in = 0x00;
				break;
			case 4:
				led_in |= 0x80;
			break;
		}
	}else
	{
		switch (key_up)
		{
			case 1:
			{
				tcs ++;
				led_in = 0x00;
				break;
			}
			case 2:
			{
				
				break;
			}
			case 3:
			{
				
				break;
			}
			case 4:
			{
				led_in |= 0x08;
				break;
			}
		}
	}
}

void lcd_Proc()
{
	if(!sta_lcd)
	{
		sprintf((char*)ptr,"    local Plat   ");
		LCD_DisplayStringLine(Line1,(u8*)ptr);
		
		sprintf((char*)ptr,"  %d  %d   ", tcs, tc[tcs] );
		LCD_DisplayStringLine(Line2,(u8*)ptr);
	}else{
		sprintf((char*)ptr,"");
		LCD_DisplayStringLine(Line0,(u8*)ptr);
	}
}


void led_Proc()
{
	
	led_dis(led_in);
}

unsigned char isRxCplt()
{
	if(dex==0)
		return 0;
	else if((dex==1)&&(rx_buf[0]=='X'))
		return 1;
	else if((dex==1)&&(rx_buf[0]=='Y'))
		return 2;
	else if((dex==3)&&(rx_buf[0]=='P')&&(rx_buf[1]=='A')&&(rx_buf[2]=='1'))//接收到3位数据 且该数据为PA1
		return 3;
	else if((dex==3)&&(rx_buf[0]=='P')&&(rx_buf[1]=='A')&&(rx_buf[2]=='4'))//接收到3位数据 且该数据为PA4
		return 4;
	else if((dex==3)&&(rx_buf[0]=='P')&&(rx_buf[1]=='A')&&(rx_buf[2]=='5'))//接收到3位数据 且该数据为PA5
		return 5;
	else if((dex==1)&&(rx_buf[0]=='#'))
		return 6;
	else//接收到其他数据
		return 7;
}

void Uart_Proc()
{
	static __IO uint32_t Uart_Tick;
	
	if(uwTick-Uart_Tick<50)
		return;
	Uart_Tick=uwTick;
	
	if(isRxCplt()==1)//返回当前频率参数
	{
		sprintf((char*)tx,"X:%d\r\n",dex);
		HAL_UART_Transmit(&huart1,tx,strlen(tx),50);
	}
	else if(isRxCplt()==2)//返回当前电压参数
	{
		sprintf((char*)tx,"Y:%d\r\n",dex);
		HAL_UART_Transmit(&huart1,tx,strlen(tx),50);
	}
	else if(isRxCplt()==3)//返回PA1通道实时测量到的频率数据
	{
		sprintf((char*)tx,"PA1:%d\r\n",dex);
		HAL_UART_Transmit(&huart1,tx,strlen(tx),50);
	}
	else if(isRxCplt()==4)//返回PA4通道当前测量到的电压数据
	{
		sprintf((char*)tx,"PA4:%d\r\n",dex);
		HAL_UART_Transmit(&huart1,tx,strlen(tx),50);
	}
	else if(isRxCplt()==5)//返回PA5通道当前测量到的电压数据
	{
		sprintf((char*)tx,"PA5:%d\r\n",dex);
		HAL_UART_Transmit(&huart1,tx,strlen(tx),50);
	}
	
	dex=0;//串口缓冲数组索引清零
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	
	rx_buf[dex++]=rx;
	HAL_UART_Receive_IT(&huart1,&rx,1);//重新开启接收中断
}

u16 adc[2];

void getADC2(void) // 获取PB15引脚的电压(对应R37电阻)
{
	u8 i;
	
//	HAL_ADC_Start(&hadc2);
//	for(i = 0; i < 2; i ++)
//	{
//		adc[i] = HAL_ADC_GetValue(&hadc2);
//		HAL_Delay(5);
//	}
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion (&hadc2,50);//等待转换完成，第二个参数表示超时时间，单位ms
	adc[0] = HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion (&hadc2,50);
	adc[1] = HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Stop(&hadc2);
	
}

uint32_t  f39 = 0, f40 = 0; //分别用来存储TIM2_CH1和TIM3_CH1的捕获频率
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	uint32_t  cc1_value = 0;
	cc1_value = __HAL_TIM_GET_COUNTER(htim);
	__HAL_TIM_SetCounter(htim,0);
	
	if(htim == &htim2) //定时器2，对应的是PA15引脚，R40
	{
		f40 = 1000000/cc1_value;
	}
	
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_TIM3_Init();
  MX_TIM17_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
	I2CInit();
	LCD_Init();
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	LCD_Clear(Black);
	led_dis(0);
	HAL_UART_Receive_IT(&huart1,&rx,1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);

	i2c_Read(e2prom,0,5);
  if((e2prom[2]==0x77)&&(e2prom[3]==0x7A)&&(e2prom[4]==0x64))//不是第一次上电
  {
		
  }
  else//是第一次上电
  {
		e2prom[2]=0x77;e2prom[3]=0x7A;e2prom[4]=0x64;//写入表示已经完成第一次上电的标志
		i2c_Write(e2prom,0,5);
  }

	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		sprintf((char*)tx,"X:%d\r\n",adc[0]);
		LCD_DisplayStringLine(Line8,(u8*)tx);
		sprintf((char*)tx,"X:%d\r\n",adc[1]);
		LCD_DisplayStringLine(Line9,(u8*)tx);

		getADC2();
		key_Proc();
		lcd_Proc();
		led_Proc();
		Uart_Proc();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
