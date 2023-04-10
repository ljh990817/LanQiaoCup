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

//lcd
u8 sta_lcd;
u8 ptr[30];
float ao2;
//led
u8 led_in;
u8 mode;

u8 pa6 = 10, pa7 = 10;
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
			if(mode)
			{
				pa6 += 10;
				if(pa6 >= 100)
					pa6 = 10;
			}
  		break;
		}
		case 3:
		{
			if(mode)
			{
				pa7 += 10;
				if(pa7 >= 100)
					pa7 = 10;
			}
  		break;
		}
		case 4:
		{
			mode = ~mode;
  		break;
		}
  }
}
float adc2;
void work_Proc()
{
//	uint16_t adc2 = 0;
	HAL_ADC_Start(&hadc2);
	adc2 = HAL_ADC_GetValue(&hadc2);
	ao2 = adc2*3.3f/4096;
//	
//	sprintf((char*)ptr,"    V:%.2f", ao2);
//		LCD_DisplayStringLine(Line8,(u8*)ptr);
//	
}

void lcd_Proc()
{
	if(!sta_lcd)
	{
		sprintf((char*)ptr,"      Data");
		LCD_DisplayStringLine(Line1,(u8*)ptr);
		sprintf((char*)ptr,"    V:%.2f", ao2);
		LCD_DisplayStringLine(Line2,(u8*)ptr);
		if(mode)
		{
			sprintf((char*)ptr,"    Mode:MANU");
			LCD_DisplayStringLine(Line4,(u8*)ptr);
		}else
		{
			sprintf((char*)ptr,"    Mode:AUTO");
			LCD_DisplayStringLine(Line4,(u8*)ptr);
		}
	}else{
		sprintf((char*)ptr,"      Para");
		LCD_DisplayStringLine(Line1,(u8*)ptr);
		if(mode)
		{
			sprintf((char*)ptr,"    PA6:%d", pa6);
			LCD_DisplayStringLine(Line2,(u8*)ptr);
			sprintf((char*)ptr,"    PA7:%d", pa7);
			LCD_DisplayStringLine(Line4,(u8*)ptr);
		}else
		{
			sprintf((char*)ptr,"    PA6:%d", pa6);
			LCD_DisplayStringLine(Line2,(u8*)ptr);
			sprintf((char*)ptr,"    PA7:%d", pa7);
			LCD_DisplayStringLine(Line4,(u8*)ptr);
		}
	}
}

void led_Proc()
{
	if(!mode)
		led_in |= 0x01;
	else
		led_in &= ~(0x01);
	
	if(!sta_lcd)
		led_in |= 0x02;
	else
		led_in &= ~(0x02);
	
	led_dis(led_in);
}

void Pwm_Proc()
{
	if(!mode)
	{
		
		if(ao2 == 0)
		{
			__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1, 10000);
			__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2, 5000);
		}else if(ao2 == 3.3f)
		{
			__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1, 0);
			__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2, 0);
		}else
		{
			pa6 = (ao2 / 3.3f) * 100;
			pa7 = (ao2 / 3.3f) * 100;
			__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1, pa6 * 100);
			__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2, pa7 * 50);
		}
	}else{
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1, pa6 * 100);
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2, pa7 * 50);
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
  MX_ADC2_Init();
  MX_TIM3_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
	LCD_Init();
	LCD_Clear(Black);
	led_dis(0);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		key_Proc();
		lcd_Proc();
		led_Proc();
		Pwm_Proc();
		work_Proc();

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 20;
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
