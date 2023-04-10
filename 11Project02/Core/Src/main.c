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
u8 mode = 0;
unsigned char tx[21],rx,rx_buf[21],dex;//串口相关变量
u32 freqPa1, freqPa2;
uint32_t  cc1_value1 = 0, cc1_value2 = 0;
u8 tc[128];
u8 tcs = 0;

u8 led_in;
u8 e2prom[5];//EEPROM存储数组
u8 PwmPA6 = 10, PwmPA7 = 10;
u8 PwmPA6o, PwmPA7o;
u8 led1_sta, led2_sta;
double sss;
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

	switch (key_up)
	{
		case 1:
		{
			sta_lcd = ~sta_lcd;
			LCD_Clear(Black);
			break;
		}
		case 2:
		{
			if(sta_lcd)
			{
				PwmPA6 += 10;
				if(PwmPA6 > 100)
					PwmPA6 = 10;
			}
			break;
		}
		case 3:
		{
			if(sta_lcd)
			{
				PwmPA7 += 10;
				if(PwmPA7 > 100)
					PwmPA7 = 10;
			}
			break;
		}
		case 4:
		{
			if(!mode)
				mode = 1;
			else
				mode = 0;
			if(mode)
			{
				__HAL_TIM_SetCompare(&htim17, TIM_CHANNEL_1, PwmPA6 - 1);
				__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, (PwmPA7 * 5) -1);
			}
			break;
		}
	}

}

double tmep;
void lcd_Proc()
{

	
	sprintf((char*)ptr,"  %d        ", freqPa1);
	LCD_DisplayStringLine(Line6,(u8*)ptr);
	sprintf((char*)ptr,"  %.2f       ", sss * 100);
	LCD_DisplayStringLine(Line7,(u8*)ptr);
	sprintf((char*)ptr,"  %.2f       ", tmep);
	LCD_DisplayStringLine(Line8,(u8*)ptr);
	
	if(!sta_lcd)
	{
		sprintf((char*)ptr,"      Data");
		LCD_DisplayStringLine(Line0,(u8*)ptr);
		sprintf((char*)ptr,"    V:%.2fV", ao2);
		LCD_DisplayStringLine(Line2,(u8*)ptr);
		if(!mode)
		{
			sprintf((char*)ptr,"    Mode:AUTO");
			LCD_DisplayStringLine(Line4,(u8*)ptr);
		}else
		{
			sprintf((char*)ptr,"    Mode:MANU");
			LCD_DisplayStringLine(Line4,(u8*)ptr);
		}
	}else{
		sprintf((char*)ptr,"      Para");
		LCD_DisplayStringLine(Line0,(u8*)ptr);
		if(mode)
		{
			sprintf((char*)ptr,"    PA6:%d%%", PwmPA6);
			LCD_DisplayStringLine(Line2,(u8*)ptr);
			sprintf((char*)ptr,"    PA7:%d%%", PwmPA7);
			LCD_DisplayStringLine(Line4,(u8*)ptr);
		}else
		{
			sprintf((char*)ptr,"    PA6:%d%%", PwmPA6o);
			LCD_DisplayStringLine(Line2,(u8*)ptr);
			sprintf((char*)ptr,"    PA7:%d%%", PwmPA7o);
			LCD_DisplayStringLine(Line4,(u8*)ptr);
		}	
	}
}

void led_Proc()
{
//	__HAL_TIM_SetAutoreload(&htim17,(1000/b/plus2)-1);
//	__HAL_TIM_SetCompare(&htim17,TIM_CHANNEL_1,(500/b/plus2));
	if(!mode)
		led_in |= 0x02;
	else
		led_in &= ~(0x02);
	
	if(!sta_lcd)
		led_in |= 0x01;
	else
		led_in &= ~(0x01);
	
	led_dis(led_in);
}

void work_Proc(void) // 获取PB15引脚的电压(对应R37电阻)
{
	uint32_t adc;
	HAL_ADC_Start(&hadc2);
	adc = HAL_ADC_GetValue(&hadc2);
	ao2 = (3.3f * adc)/4096;
	tmep = adc * 1.0 / 4906 * 100;
	
	
	PwmPA6o = adc * 1.0 / 4906 * 100;
	__HAL_TIM_SetAutoreload(&htim3, 100 - 1);
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, PwmPA6o - 1);

}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	
	cc1_value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4) + 1;
	cc1_value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3) + 1;
	__HAL_TIM_SetCounter(htim, 0);
	
	if(htim == &htim1) 
	{
		freqPa1 = 1000000 / cc1_value1;
		sss = (cc1_value2 * 1.0) / cc1_value1;
	}
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);
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
  MX_ADC2_Init();
  MX_TIM3_Init();
  MX_TIM17_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	I2CInit();
	LCD_Init();
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	LCD_Clear(Black);
	led_dis(0);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
