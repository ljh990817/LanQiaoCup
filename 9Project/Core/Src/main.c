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

#define Pressed			1
#define UnPressed		0
#define Long_Pressed	800
uint8_t KEY_FLAG = 0;
#define Key_Long_OK		2
#define KEY_Short_OK	3
#define KEY_Error		4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
u8 key_val,key_down,key_up,key_old;
u8 sta_lcd;
u8 ptr[30];
u8 led_in;
u8 save_id = 0;
u8 save_set[5] = {0x01, 0x04, 0x07, 0x10, 0x14};
u8 t1 = 10;
u8 k2_state, k3_state, k4_state = Key_Long_OK;
u8 hms, k2tt = 0, k4tt = 0, k3tt = 0;
u8 hour, min, sec;
uint32_t GetTick, GetTick1, GetTick2, GetTick3;
u16 times = 0;
u8 start1;
u8 begin = 0;

uint8_t KEYB2_FLAG = 0;
uint8_t KEYB4_FLAG = 0;
uint8_t KEYB4_FLAGS = 0;
uint8_t temp = 0;

uint16_t KEY_TIME = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t x24c02_read(uint8_t address)
{
	unsigned char val;
	
	I2CStart(); 
	I2CSendByte(0xa0);
	I2CWaitAck(); 
	
	I2CSendByte(address);
	I2CWaitAck(); 
	
	I2CStart();
	I2CSendByte(0xa1); 
	I2CWaitAck();
	val = I2CReceiveByte(); 
	I2CWaitAck();
	I2CStop();
	
	return(val);
}

void x24c02_write(unsigned char address,unsigned char info)
{
	
	I2CStart(); 
	I2CSendByte(0xa0); 
	I2CWaitAck(); 
	
	I2CSendByte(address);	
	I2CWaitAck(); 
	I2CSendByte(info); 
	I2CWaitAck(); 
	I2CStop();
	HAL_Delay(5);
}

u32 qwe = 800, nn;

u32 ktci = 0;
uint8_t Key_Scan(GPIO_TypeDef *GPIOx, uint16_t GPIO_PIN)
{
	if(HAL_GPIO_ReadPin(GPIOx,GPIO_PIN) == RESET)
	{
		KEY_TIME = 0;
		GetTick = uwTick;
		nn = uwTick;
		while(HAL_GPIO_ReadPin(GPIOx,GPIO_PIN) == RESET)
		{
			KEY_TIME = uwTick - GetTick;
			if(k2tt && GPIOx == GPIOB && GPIO_PIN == GPIO_PIN_2 && KEY_TIME > 800)
			{
				if(uwTick - nn > 1000)
				{
					nn = uwTick;
					if(hms == 1)
					{
						hour += 1;
						if(hour > 24)
							hour = 0;
						LCD_SetTextColor(Yellow);
						LCD_DisplayChar(Line4, 319 - (16 * 7), (hour / 10) + 48);
						LCD_DisplayChar(Line4, 319 - (16 * 8), (hour % 10) + 48);
						LCD_SetTextColor(White);
					}else if(hms == 2)
					{
						min += 1;
						if(min > 60)
							min = 0;
						LCD_SetTextColor(Yellow);
						LCD_DisplayChar(Line4, 319 - (16 * 10), (min / 10) + 48);
						LCD_DisplayChar(Line4, 319 - (16 * 11), (min % 10) + 48);
						LCD_SetTextColor(White);
					}else if(hms == 0)
					{
						sec += 1;
						if(sec > 60)
							sec = 0;
						LCD_SetTextColor(Yellow);
						LCD_DisplayChar(Line4, 319 - (16 * 13), (sec / 10) + 48);
						LCD_DisplayChar(Line4, 319 - (16 * 14), (sec % 10) + 48);
						LCD_SetTextColor(White);
					}
				}	
				
			}
		}
		KEY_FLAG = Pressed;
	}

	if(KEY_FLAG == Pressed && KEY_TIME <= Long_Pressed)
	{
		KEY_FLAG = UnPressed;
		return KEY_Short_OK;
	}
	if(KEY_FLAG == Pressed && KEY_TIME > Long_Pressed)
	{
		KEY_FLAG = UnPressed;
		return Key_Long_OK;
	}
	
	return KEY_Error;
}

u8 key_scan()
{
	if((KEY1 == 0))
	{
		HAL_Delay(10);
		if(KEY1 == 0) return 1;
	}
	return 0;
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
	
	if(key_up == 1)
	{
		begin = 1;
		save_id ++;
		if(save_id == 5)
			save_id = 0;
	}
}

void lcd_Proc()
{
	sprintf((char*)ptr,"   No %d", save_id);
	LCD_DisplayStringLine(Line1,(u8*)ptr);
	if(k2tt == 0)
	{
		sprintf((char*)ptr,"       %02d:%02d:%02d", hour, min, sec);
		LCD_DisplayStringLine(Line4,(u8*)ptr);
	}else
	{
		if(hms == 1)
		{
			LCD_SetTextColor(Yellow);
			LCD_DisplayChar(Line4, 319 - (16 * 7), (hour / 10) + 48);
			LCD_DisplayChar(Line4, 319 - (16 * 8), (hour % 10) + 48);
			LCD_SetTextColor(White);
			LCD_DisplayChar(Line4, 319 - (16 * 9), 58);
			LCD_DisplayChar(Line4, 319 - (16 * 10), (min / 10) + 48);
			LCD_DisplayChar(Line4, 319 - (16 * 11), (min % 10) + 48);
			LCD_DisplayChar(Line4, 319 - (16 * 12), 58);
			LCD_DisplayChar(Line4, 319 - (16 * 13), (sec / 10) + 48);
			LCD_DisplayChar(Line4, 319 - (16 * 14), (sec % 10) + 48);
			
		}else if(hms == 2)
		{
			LCD_DisplayChar(Line4, 319 - (16 * 7), (hour / 10) + 48);
			LCD_DisplayChar(Line4, 319 - (16 * 8), (hour % 10) + 48);
			LCD_DisplayChar(Line4, 319 - (16 * 9), 58);
			LCD_SetTextColor(Yellow);
			LCD_DisplayChar(Line4, 319 - (16 * 10), (min / 10) + 48);
			LCD_DisplayChar(Line4, 319 - (16 * 11), (min % 10) + 48);
			LCD_SetTextColor(White);
			LCD_DisplayChar(Line4, 319 - (16 * 12), 58);
			LCD_DisplayChar(Line4, 319 - (16 * 13), (sec / 10) + 48);
			LCD_DisplayChar(Line4, 319 - (16 * 14), (sec % 10) + 48);
		}else if(hms == 0)
		{
			LCD_DisplayChar(Line4, 319 - (16 * 7), (hour / 10) + 48);
			LCD_DisplayChar(Line4, 319 - (16 * 8), (hour % 10) + 48);
			LCD_DisplayChar(Line4, 319 - (16 * 9), 58);
			LCD_DisplayChar(Line4, 319 - (16 * 10), (min / 10) + 48);
			LCD_DisplayChar(Line4, 319 - (16 * 11), (min % 10) + 48);
			LCD_DisplayChar(Line4, 319 - (16 * 12), 58);
			LCD_SetTextColor(Yellow);
			LCD_DisplayChar(Line4, 319 - (16 * 13), (sec / 10) + 48);
			LCD_DisplayChar(Line4, 319 - (16 * 14), (sec % 10) + 48);
			LCD_SetTextColor(White);
		}
	}
	
	if(begin == 0)
	{
		sprintf((char*)ptr,"      StandBy   ");
		LCD_DisplayStringLine(Line8,(u8*)ptr);
	}
	
	if(k4_state == Key_Long_OK)
	{
		sprintf((char*)ptr,"      StandBy   ");
		LCD_DisplayStringLine(Line8,(u8*)ptr);
	}else if(k4_state == KEY_Short_OK && k4tt == 0)
	{
		sprintf((char*)ptr,"      Pause     ");
		LCD_DisplayStringLine(Line8,(u8*)ptr);
	}else if(k4_state == KEY_Short_OK && k4tt == 1)
	{
		sprintf((char*)ptr,"      Running   ");
		LCD_DisplayStringLine(Line8,(u8*)ptr);
	}else if(k2_state == KEY_Short_OK)
	{
		sprintf((char*)ptr,"      Setting   ");
		LCD_DisplayStringLine(Line8,(u8*)ptr);
	}else if(k2_state == Key_Long_OK)
	{
		sprintf((char*)ptr,"      StandBy   ");
		LCD_DisplayStringLine(Line8,(u8*)ptr);
	}
}


void led_Proc()
{
	if(k4tt == 0)
	{
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 0);
		led_in = 0x00;
	}
	
	if(k4tt == 1)
	{ 
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 800);
		if(uwTick - GetTick2 > 500)
		{
			GetTick2 = uwTick;
			led_in ^= 0x01;
		}
	}
	
	led_dis(led_in);
}

void key2_work()
{
	
	k2_state = Key_Scan(GPIOB, GPIO_PIN_1);
	if(k2_state == KEY_Short_OK)
	{
		begin = 1;
		k2tt = 1;
		hms ++;
		if(hms == 3)
			hms = 0;
	}else if(k2_state == Key_Long_OK)
	{
		begin = 1;
		k2tt = 0;
		x24c02_write(0x20, save_id);
		x24c02_write(save_set[save_id], hour);
		x24c02_write(save_set[save_id] + 0x01, min);
		x24c02_write(save_set[save_id] + 0x02, sec);
	}
}

void key3_work()
{
	
	k3_state = Key_Scan(GPIOB, GPIO_PIN_2);
	if(k3_state == KEY_Short_OK)
	{
		begin = 1;
		if(k2tt == 1)
		{
			if(hms == 1)
			{
				hour ++;
				if(hour > 24)
					hour = 0;
			}else if(hms == 2)
			{
				min ++;
				if(min > 60)
					min = 0;
			}else if(hms == 0)
			{
				sec ++;
				if(sec > 60)
					sec = 0;
			}
		}
	}
}

void secpp()
{
	if(k4tt == 1)
	{
		if(uwTick - GetTick1 > 1000)
		{
			sec += 1;
			if(sec > 60)
			{
				sec = 0;
				min ++;
			}
			if(min > 60)
			{
				min = 0;
				hour ++;
			}
			if(hour > 24)
			{
				hour = 0;
			}
			GetTick1 = uwTick;
		}
	}
}

void key4_work()
{
	
	k4_state = Key_Scan(GPIOA, GPIO_PIN_0);
	if(k4_state == KEY_Short_OK)
	{
		begin = 1;
		k4tt = !k4tt;
		GetTick2 = uwTick;
		GetTick1 = uwTick;
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
  /* USER CODE BEGIN 2 */
	I2CInit();
	LCD_Init();
	
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	led_dis(0);
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
	
	start1 = x24c02_read(0x00);
	x24c02_write(0x00, 0x44);
	
	if(start1 == 0x44)
	{
		save_id = x24c02_read(0x20);
		hour = x24c02_read(save_set[save_id]);
		min = x24c02_read(save_set[save_id] + 0x01);
		sec = x24c02_read(save_set[save_id] + 0x02);
	}else
	{
		x24c02_write(0x20, save_id);
		x24c02_write(save_set[save_id], hour);
		x24c02_write(save_set[save_id] + 0x01, min);
		x24c02_write(save_set[save_id] + 0x02, sec);
	}
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		key_Proc();
		key2_work();
		key3_work();
		key4_work();
		led_Proc();
		lcd_Proc();
		secpp();
		
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
