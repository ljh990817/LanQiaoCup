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
u32 freqPa1, freqPa2;
uint32_t  cc1_value1 = 0, cc1_value2 = 0;
unsigned char tx[21],rx,rx_buf[21],dex;//串口相关变量
u8 rk_mode;
u8 tc[128];
u8 tcs = 0;
u8 led2_first = 0;
u8 led_in;
u8 e2prom[5];//EEPROM存储数组
u8 PwmSD;
u8 R_sta = 1, K_sta = 1;
u8 PwmMode;
u16 PwmNum;
u8 Pwmzb;
u8 bbbb = 0, ttt = 0;
double GPv = 0.0, DPv = 0.0;
	
__IO uint32_t LED2_Tick;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

void pwm_proc(void)
{
	__IO uint32_t Key22_Tick = LED2_Tick;
	u8 ii = 100;
	u8 i2 = 200;
	
	if((uwTick - LED2_Tick < 5000) && ttt == 1)
	{
		if(bbbb == 0)
		{
			if(uwTick - Key22_Tick > 100)
			{
				Key22_Tick = uwTick;
				ii += 2;
				
				__HAL_TIM_SetAutoreload(&htim1, i2-1);					
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, (Pwmzb / 100 * (i2 - 1)) - 1);

			}
		}else if(bbbb == 1)
		{	
			
			if(uwTick - Key22_Tick > 100)
			{
				Key22_Tick = uwTick;
				i2 -= 2;
				__HAL_TIM_SetAutoreload(&htim1, ii-1);					
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, (Pwmzb /100 * (ii - 1)) - 1);

			}
		}
	}

	if(uwTick - LED2_Tick > 5000)
	{
		if(PwmMode == 0)
		{
			__HAL_TIM_SetAutoreload(&htim1, 200 - 1);					
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, (Pwmzb * 2) - 1);
		
		}else
		{
			__HAL_TIM_SetAutoreload(&htim1, 100 - 1);					
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, Pwmzb - 1);
		}
		
		ttt = 0;
	}
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

	if(uwTick - DKey_Tick > 2000)
	{
		switch(key_up)
		{
			case 4:
				if(sta_lcd == 0)
				{
					if(PwmSD == 0)
						PwmSD = 1; // 锁定
				}
			break;
		}
	}else
	{
		switch (key_up)
		{
			case 1:
			{
				sta_lcd ++;
				if(sta_lcd == 3)
					sta_lcd = 0;
				
				if(sta_lcd == 1)
					rk_mode = 0;
				
				LCD_Clear(Black);
				break;
			}
			case 2:
			{
				if(sta_lcd == 0)
				{	
					if((uwTick - LED2_Tick > 5000) | (led2_first == 0))
					{
						LED2_Tick = uwTick;
						ttt = 1;
						if(PwmMode == 0)// 1高频 0 低频
						{
							PwmMode = 1;
//							__HAL_TIM_SetAutoreload(&htim2, 100-1);
//							__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2, Pwmzb - 1);
							bbbb = 1;

						}else if(PwmMode == 1)
						{
							PwmMode = 0;
							bbbb = 0;                    
//							__HAL_TIM_SetAutoreload(&htim2, 200-1);
//							__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2, (Pwmzb - 1) * 2);
						}
						led2_first = 1;
						PwmNum ++;
					}
				}else if(sta_lcd == 1)
				{
					if(rk_mode == 0) 
					{
						rk_mode = 1;
					}
					else if(rk_mode == 1)
					{
						rk_mode = 0;
					}
				}
				break;
			}
			case 3:
			{
				if(sta_lcd == 1)
				{
					if(rk_mode == 0)
					{
						R_sta ++;
						if(R_sta > 10)
							R_sta = 1;
					}
					else if(rk_mode == 1)
					{
						K_sta ++;
						if(K_sta > 10)
							K_sta = 1;
					}
				}
				break;
			}
			case 4:
			{
				if(sta_lcd == 1)
				{
					if(rk_mode == 0)
					{
						R_sta --;
						if(R_sta < 1)
							R_sta = 10;
					}
					else if(rk_mode == 1)
					{
						K_sta --;
						if(K_sta < 1)
							K_sta = 10;
					}
				}else if(sta_lcd == 0)
				{
					if(PwmSD == 1)
						PwmSD = 0;
				}
				break;
			}
		}
	}
}



void lcd_Proc()
{
	
	double v_sta = (freqPa1 * 3.14 * 2 * R_sta) / 100.0 * K_sta;
	
	if(PwmMode == 0)
	{
		if(v_sta > DPv)
			DPv = v_sta;
	}else if(PwmMode == 1)
	{
		if(v_sta > GPv)
			GPv = v_sta;
	}

	if(sta_lcd == 0)
	{
		sprintf((char*)ptr,"        DATA");
		LCD_DisplayStringLine(Line1,(u8*)ptr);
		if(PwmMode) // 1 为高频
		{
			sprintf((char*)ptr,"     M=H");
		}else  // 0 为低频
		{
			sprintf((char*)ptr,"     M=L");
		}
		LCD_DisplayStringLine(Line3,(u8*)ptr);
		sprintf((char*)ptr,"     P=%d%%   ", Pwmzb);
		LCD_DisplayStringLine(Line4,(u8*)ptr);
		sprintf((char*)ptr,"     V=%d   ", freqPa1);
		LCD_DisplayStringLine(Line5,(u8*)ptr);
	}else if(sta_lcd == 1){
		sprintf((char*)ptr,"        PARA");
		LCD_DisplayStringLine(Line1,(u8*)ptr);
		sprintf((char*)ptr,"     R=%d  ", R_sta);
		LCD_DisplayStringLine(Line3,(u8*)ptr);
		sprintf((char*)ptr,"     K=%d  ", K_sta);
		LCD_DisplayStringLine(Line4,(u8*)ptr);
	}else if(sta_lcd == 2)
	{
		sprintf((char*)ptr,"        RECD");
		LCD_DisplayStringLine(Line1,(u8*)ptr);
		sprintf((char*)ptr,"     N=%d   ", PwmNum);
		LCD_DisplayStringLine(Line3,(u8*)ptr);
		sprintf((char*)ptr,"     MH=%.1f   ", DPv);
		LCD_DisplayStringLine(Line4,(u8*)ptr);
		sprintf((char*)ptr,"     ML=%.1f   ", GPv);
		LCD_DisplayStringLine(Line5,(u8*)ptr);
	}
}

void led_Proc()
{
//	__HAL_TIM_SetAutoreload(&htim17,(1000/b/plus2)-1);
//	__HAL_TIM_SetCompare(&htim17,TIM_CHANNEL_1,(500/b/plus2));
	static __IO uint32_t LED2_Tick2;
	
	if(sta_lcd == 0)
		led_in |= 0x01;
	else 
		led_in &= ~(0x01);
	
	// LED2 
	if((uwTick - LED2_Tick < 5000) && LED2_Tick != 0)
	{
		if(uwTick - LED2_Tick2 > 100)
		{
			led_in ^= 0x02;
			LED2_Tick2 = uwTick;
		}
	}else
	{
		led_in &= ~(0x02);
	}
	
	if(PwmSD == 1)
		led_in |= 0x04;
	else
		led_in &= ~(0x04);
	
	led_dis(led_in);
}


void work_Proc(void) // 获取PB15引脚的电压(对应R37电阻)
{
	uint16_t adc = 0;
	HAL_ADC_Start(&hadc2);
	adc = HAL_ADC_GetValue(&hadc2);
	ao2 = (3.3 * adc)/4096;
	
	if(PwmSD == 0)
	{
		if(ao2 < 1.0000001)
		{
			Pwmzb = 10;
		}else if(ao2 > 3.000001)
		{
			Pwmzb = 85;
		}else
		{
			Pwmzb = 10 + ((ao2 - 1) / 2.0) * 75;
		}
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	uint32_t  cc1_value = 0;
	cc1_value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
	__HAL_TIM_SetCounter(htim, 0);
	
	if(htim == &htim3) 
	{
			freqPa1 = 1000000 / cc1_value;
	}
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);

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
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

	LCD_Init();
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	LCD_Clear(Black);
	led_dis(0);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		pwm_proc();
		work_Proc();
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
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
