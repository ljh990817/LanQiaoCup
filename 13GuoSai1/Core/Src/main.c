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
u8 key_val,key_down,key_up,key_old;
double ao1, ao2;
u8 sta_lcd;
u8 ptr[30];

unsigned char tx[21],rx,rx_buf[21],dex;//串口相关变量

u8 tc[128];
u8 tcs = 0;

u8 led_in;
u8 e2prom[5];//EEPROM存储数组

double ao1, ao2;
	
typedef struct
{
	double maxn;
	double minn;
	double tmmin;
	double avgr;
	int record;
	double all[101];
	double sum;
}Vol;

Vol vol4, vol5;

u32 freqPa1, freqPa7;
u8 freqX = 1, VolY = 1;
u8 showN;
u8 lcdf, ff;
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

void getADC(void) 
{
	uint16_t adc = 0;
	HAL_ADC_Start(&hadc1);
	adc = HAL_ADC_GetValue(&hadc1);
	ao1 = (3.3 * adc)/4096;
//	HAL_Delay(5);
	HAL_ADC_Start(&hadc2);
	adc = HAL_ADC_GetValue(&hadc2);
	ao2 = (3.3 * adc)/4096;
//	HAL_ADC_Stop(&hadc2);
}

u8 i, j = 0;
void fresh_lcd()
{
	getADC();
	vol4.all[vol4.record] = ao1;
	vol5.all[vol5.record] = ao2;
	
	if(vol4.record == 0)
	{
		vol4.minn = 3.3;
		vol4.maxn = 0;
	}
	
	if(vol5.record == 0)
	{
		vol5.minn = 3.3;
		vol5.maxn = 0;
	}

	vol4.maxn = vol4.maxn < vol4.all[vol4.record] ? vol4.all[vol4.record] : vol4.maxn;
	vol5.maxn = vol5.maxn < vol5.all[vol5.record] ? vol5.all[vol5.record] : vol5.maxn;

	vol4.minn = vol4.minn > vol4.all[vol4.record] ? vol4.all[vol4.record] : vol4.minn;
	vol5.minn = vol5.minn > vol5.all[vol5.record] ? vol5.all[vol5.record] : vol5.minn;
	
//	for(j = 0; j < vol4.record; j ++)
//		vol4.sum += vol4.all[j];
	vol4.sum += vol4.all[vol4.record];
	vol4.avgr = vol4.sum / (vol4.record * 1.0);
//	vol4.sum = 0.0;
	vol4.record ++;
	
//	for(j = 0; j < vol5.record; j ++)
//		vol5.sum += vol5.all[j];
	vol5.sum += vol5.all[vol5.record];
	vol5.avgr = vol5.sum / (vol5.record * 1.0);
//	vol5.sum = 0.0;
	vol5.record++;
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
			case 4:
				if(!showN)
				{
					for(i = 0; i < vol4.record; i ++)
						vol4.all[i] = 0;
					vol4.record = 0;
					vol4.maxn = 0;
					vol4.minn = 0;
					vol4.avgr = 0;
				}else
				{
					for(i = 0; i < vol5.record; i ++)
						vol5.all[i] = 0;
					vol5.record = 0;
					vol5.maxn = 0;
					vol5.minn = 0;
					vol5.avgr = 0;
				}
			break;
		}
	}
	else
	{
		switch (key_up)
		{
			case 1:
			{
				sta_lcd ++;
				if(sta_lcd == 3)
					sta_lcd = 0;
				LCD_Clear(Black);
				break;
			}
			case 2:
			{
				if(sta_lcd == 1)
				{
					freqX ++;
					if(freqX == 5)
						freqX = 1;
//					i2c_Write(e2prom,0,5);
				}
				break;
			}
			case 3:
			{
				if(sta_lcd == 1)
				{
					VolY ++;
					if(VolY == 5)
						VolY = 1;
//					i2c_Write(e2prom,0,5);
				}
				break;
			}
			case 4:
			{
				if(sta_lcd == 0)
				{
					fresh_lcd();
				}else if(sta_lcd == 1)
				{
					ff = ~ff;
					if(!ff)
					{
						freqPa7 = freqPa1 * freqX;
						__HAL_TIM_SET_AUTORELOAD(&htim3,freqPa7 - 1);
						__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,0.5*freqPa7);
					}else
					{
						freqPa7 = freqPa1 / freqX;
						__HAL_TIM_SET_AUTORELOAD(&htim3,freqPa7 - 1);
						__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,0.5*freqPa7);
					}
				}else if(sta_lcd == 2)
				{
					showN = ~showN;
				}
				break;
			}
		}
	}
}
	
void lcd_Proc()
{
	if(sta_lcd == 0)
	{
		sprintf((char*)ptr,"        DATA");
		LCD_DisplayStringLine(Line1,(u8*)ptr);
		sprintf((char*)ptr,"     PA4=%.2lf", vol4.all[vol4.record - 1]);
		LCD_DisplayStringLine(Line3,(u8*)ptr);
		sprintf((char*)ptr,"     PA5=%.2lf", vol5.all[vol5.record - 1]);
		LCD_DisplayStringLine(Line4,(u8*)ptr);
		sprintf((char*)ptr,"     PA1=%d      ", freqPa1);
		LCD_DisplayStringLine(Line5,(u8*)ptr);
	}else if(sta_lcd == 1){
		sprintf((char*)ptr,"        PARA");
		LCD_DisplayStringLine(Line1,(u8*)ptr);
		sprintf((char*)ptr,"     X=%d", freqX);
		LCD_DisplayStringLine(Line3,(u8*)ptr);
		sprintf((char*)ptr,"     Y=%d", VolY);
		LCD_DisplayStringLine(Line4,(u8*)ptr);
	}else if(sta_lcd == 2){
		if(!showN)
		{
			sprintf((char*)ptr,"        REC-PA4");
			LCD_DisplayStringLine(Line1,(u8*)ptr);
			sprintf((char*)ptr,"     N=%d", vol4.record);
			LCD_DisplayStringLine(Line3,(u8*)ptr);
			sprintf((char*)ptr,"     A=%.2lf", vol4.maxn);
			LCD_DisplayStringLine(Line4,(u8*)ptr);
			sprintf((char*)ptr,"     T=%.2lf", vol4.minn);
			LCD_DisplayStringLine(Line5,(u8*)ptr);
			sprintf((char*)ptr,"     H=%.2lf", vol4.avgr);
			LCD_DisplayStringLine(Line6,(u8*)ptr);
		}else
		{
			sprintf((char*)ptr,"        REC-PA5");
			LCD_DisplayStringLine(Line1,(u8*)ptr);
			sprintf((char*)ptr,"     N=%d", vol5.record);
			LCD_DisplayStringLine(Line3,(u8*)ptr);
			sprintf((char*)ptr,"     A=%.2lf", vol5.maxn);
			LCD_DisplayStringLine(Line4,(u8*)ptr);
			sprintf((char*)ptr,"     T=%.2lf", vol5.minn);
			LCD_DisplayStringLine(Line5,(u8*)ptr);
			sprintf((char*)ptr,"     H=%.2lf", vol5.avgr);
			LCD_DisplayStringLine(Line6,(u8*)ptr);
		}
	}
}

void led_Proc()
{
	static __IO uint32_t LED_Tick;
//	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, 20);
	
	if(ff == 0) // 0 倍频 1 分频
	{
		led_in |= 0x01;
		led_in &= ~(0x02);
	}else
	{
		led_in |= 0x02;
		led_in &= ~(0x01);
	}
	
	if(vol4.all[vol4.record - 1] > vol5.all[vol5.record - 1] * VolY)
	{
		if(uwTick - LED_Tick > 100)
		{
			LED_Tick = uwTick;
			led_in ^= 0x04;
		}
	}else
	{
		led_in &= ~(0x04);
	}
	
	if(!lcdf)
		led_in |= 0x08;
	else
		led_in &= ~(0x08);
	
	led_dis(led_in);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	uint32_t  cc1_value = 0;
//	cc1_value = __HAL_TIM_GET_COUNTER(htim);
//	__HAL_TIM_SetCounter(htim,0);
	cc1_value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
	__HAL_TIM_SetCounter(htim, 0);
	
	if(htim == &htim2) 
	{
			freqPa1 = 1000000 / cc1_value;
//		freqPa1 = (80000000 / 80)/cc1_value;
	}
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
}

unsigned char isRxCplt()
{
	if(dex==0)
		return 0;
	else if((dex==1)&&(rx_buf[0]=='X'))
		return 1;
	else if((dex==1)&&(rx_buf[0]=='Y'))
		return 2;
	else if((dex==3)&&(rx_buf[0]=='P')&&(rx_buf[1]=='A')&&(rx_buf[2]=='1'))
		return 3;
	else if((dex==3)&&(rx_buf[0]=='P')&&(rx_buf[1]=='A')&&(rx_buf[2]=='4'))
		return 4;
	else if((dex==3)&&(rx_buf[0]=='P')&&(rx_buf[1]=='A')&&(rx_buf[2]=='5'))
		return 5;
	else if((dex==1)&&(rx_buf[0]=='#'))
		return 6;
	else
		return 7;
}

void Uart_Proc()
{
	static __IO uint32_t Uart_Tick;
	
	if(uwTick-Uart_Tick<50)
		return;
	Uart_Tick=uwTick;
	
	
	if(isRxCplt()==1)
	{
		sprintf((char*)tx,"X:%d\r\n",freqX);
		HAL_UART_Transmit(&huart1,tx,sizeof(tx),50);
	}else if(isRxCplt()==2)
	{
		sprintf((char*)tx,"Y:%d\r\n",VolY);
		HAL_UART_Transmit(&huart1,tx,sizeof(tx),50);
	}else if(isRxCplt()==3)
	{
		sprintf((char*)tx,"PA1:%d\r\n",freqPa1);
		HAL_UART_Transmit(&huart1,tx,sizeof(tx),50);
	}else if(isRxCplt()==4)
	{
		sprintf((char*)tx,"PA4:%.2lf\r\n",vol4.all[vol4.record]);
		HAL_UART_Transmit(&huart1,tx,sizeof(tx),50);
	}else if(isRxCplt()==5)
	{
		sprintf((char*)tx,"PA5:%.2lf\r\n",vol5.all[vol5.record]);
		HAL_UART_Transmit(&huart1,tx,sizeof(tx),50);
	}else if(isRxCplt()==6)
	{
		lcdf = ~lcdf;
	}
	
	dex=0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	rx_buf[dex++]=rx;
	HAL_UART_Receive_IT(&huart1,&rx,1);
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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	I2CInit();
	LCD_Init();
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	LCD_Clear(Black);
	led_dis(0);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_UART_Receive_IT(&huart1,&rx,1);
	getADC(); 

//	i2c_Read(e2prom,0,5);
//	if((e2prom[2]==0x77)&&(e2prom[3]==0x7A)&&(e2prom[4]==0x64))
//	{
//		freqX = e2prom[0];
//		VolY  = e2prom[1];
//	}
//	else
//	{
//		e2prom[2]=0x77;e2prom[3]=0x7A;e2prom[4]=0x64;
//		e2prom[0] = freqX; e2prom[1] = VolY;
//		i2c_Write(e2prom,0,5);
//	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		Uart_Proc();
		key_Proc();
		lcd_Proc();
		led_Proc();
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
