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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "i2c_hal.h"
#include "lcd.h"
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
int key[3] = {-1, -1, -1};
u8 rightkey[3] = {1, 2, 3};
u16 freq = 2000;
u8 zb = 50, err_num, err_n;
__IO uint32_t Led1_Tick, Led2_Tick1, Led2_Tick2, Pwm_Tick;
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
			if(!sta_lcd)
			{
				key[0] ++;
				if(key[0] == 10)
					key[0] = 0;
				break;
			}
		}
		case 2:
		{
			if(!sta_lcd)
			{
				key[1] ++;
				if(key[1] == 10)
					key[1] = 0;
			}
			break;
		}
		case 3:
		{
			if(!sta_lcd)
			{
				key[2] ++;
				if(key[2] == 10)
					key[2] = 0;
			}
			break;
		}
		case 4:
		{
			if(key[0] == rightkey[0] && key[1] == rightkey[1] && key[2] == rightkey[2])
			{
				sta_lcd = 1;
				Led1_Tick = uwTick;
				__HAL_TIM_SetAutoreload(&htim2, 2000);
				__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,(200));
				freq = 2000;
				zb = 10;
			}else
			{
				key[0] = -1;
				key[1] = -1;
				key[2] = -1;
				err_num ++;
				if(err_num > 2)
				{
					err_n = 1;
					Led2_Tick1 = uwTick;
					Led2_Tick2 = uwTick;
				}
			}
			LCD_Clear(Black);
			break;
		}
	}

}

void lcd_Proc()
{
	if(!sta_lcd)
	{
		sprintf((char*)ptr,"       PSD");
		LCD_DisplayStringLine(Line1,(u8*)ptr);
		if(key[0] == -1)
			sprintf((char*)ptr,"    B1:@");
		else
			sprintf((char*)ptr,"    B1:%d", key[0]);
		LCD_DisplayStringLine(Line3,(u8*)ptr);
		if(key[1] == -1)
			sprintf((char*)ptr,"    B2:@");
		else
			sprintf((char*)ptr,"    B2:%d", key[1]);
		LCD_DisplayStringLine(Line4,(u8*)ptr);
		if(key[2] == -1)
			sprintf((char*)ptr,"    B3:@");
		else
			sprintf((char*)ptr,"    B3:%d", key[2]);
		LCD_DisplayStringLine(Line5,(u8*)ptr);
		
	}else{
		sprintf((char*)ptr,"       STA");
		LCD_DisplayStringLine(Line1,(u8*)ptr);
		sprintf((char*)ptr,"    F:%dHz", freq);
		LCD_DisplayStringLine(Line3,(u8*)ptr);
		sprintf((char*)ptr,"    D:%d%%", zb);
		LCD_DisplayStringLine(Line4,(u8*)ptr);
	}
}

void led_Proc()
{
//	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, 20);
	if(sta_lcd)
	{
		if(uwTick - Led1_Tick < 5000)
		{
			led_in |= 0x01;
		}else
		{
			led_in &= ~(0x01); 
			__HAL_TIM_SetAutoreload(&htim2, 1000);
			__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,(500));
			freq = 1000;
			zb = 50;
			sta_lcd = 0;
			key[0] = -1;
			key[1] = -1;
			key[2] = -1;
			LCD_Clear(Black);
		}
	}else
	{
		if(err_n)
		{
			if(uwTick - Led2_Tick1 < 5000)
			{
				if(uwTick - Led2_Tick2 > 100)
				{
					led_in ^= 0x02;
					Led2_Tick2 = uwTick;
				}
			}else
			{
				led_in &= ~(0x02);
			}
		}
	}
	
	led_dis(led_in);
}

unsigned char isRxCplt()
{
	if((dex==7)&&(rx_buf[3]=='-'))
		return 1;
	else
		return 0;
}

void Uart_Proc()
{
	static __IO uint32_t Uart_Tick;
	
	if(uwTick-Uart_Tick<50)
		return;
	Uart_Tick=uwTick;
	
	if(isRxCplt())
	{
		if(rightkey[0] == (rx_buf[0] - '0') && rightkey[1] == (rx_buf[1] - '0') && rightkey[2] == (rx_buf[2] - '0'))
		{
			rightkey[0] = (rx_buf[4] - '0');
			rightkey[1] = (rx_buf[5] - '0');
			rightkey[2] = (rx_buf[6] - '0');
		}
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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	I2CInit();
	LCD_Init();
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	LCD_Clear(Black);
	led_dis(0);
//	HAL_TIM_IC_Start_IT
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_UART_Receive_IT(&huart1,&rx,1);
	
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
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
