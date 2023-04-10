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
__IO uint32_t uwTick_Led_Set_Point = 0;
__IO uint32_t uwTick_Led_Set_Point1 = 0;
//led
u8 led_in;
u8 key_uu;
u8 start1, start2;
u8 flag_u;
u8 state;
u8 first;

u8 uled = 0, lled = 1;

uint16_t LED_Arr[9]={0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};

double MaxStadVol = 2.4;
double MinStadVOl = 1.2;

double ao2;
double Vol = 3.22;
double MaxVol = 2.4;
double MinVOl = 1.2;
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

//向24C02写数据
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
	HAL_Delay(10);
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
	key_val = key_scan();
	key_down = key_val&(key_val^key_old);
	key_up = ~key_val&(key_val^key_old);
	key_old = key_val;
	
	switch (key_up)
  {
  	case 1:
		{
			sta_lcd = !sta_lcd;
			LCD_Clear(Black);
  		break;
		}
		case 2:
		{
			if(sta_lcd)
			{
				key_uu ++;
				if(key_uu == 4)
					key_uu = 0;
			}
  		break;
		}
		case 3:
		{
			if(sta_lcd)
			{
				if(key_uu == 0) //高电压
				{
					MaxVol = MaxVol+0.3;
					if(MaxVol > 3.300000000000001)
					{
						MaxVol=3.3;
					}
					x24c02_write(0x01, MaxVol * 10);
				}else if(key_uu == 1) //低电压
				{
					MinVOl = MinVOl+0.3;
					if(MinVOl > 3.300000000000001)
					{
						MinVOl=3.3;
					}
					x24c02_write(0x02, MinVOl * 10);
				}
				else if(key_uu == 2) //Uper
				{
					led_in &= 0x00;
					if(uled == 7)
						return;
					
					if(uled + 1 == lled)
					{
						if(uled + 2 <= 7)
						{
							uled = uled + 2;
						}
					}else
						uled ++;
						x24c02_write(0x03, uled);

				}else
				{
					led_in &= 0x00;
					if(lled == 7)
						return;
					
					if(lled + 1 == uled)
					{
						if(lled + 2 <= 7)
						{
							lled = lled + 2;
						}
					}else
						lled ++;
					x24c02_write(0x04, lled);
				}
			}
  		break;
		}
		case 4:
		{
			if(sta_lcd)
			{
				if(key_uu == 0) //高电压
				{
					MaxVol = MaxVol - 0.3;
					if(MaxVol < 0.000000000000001)
					{
						MaxVol=0;
					}
					x24c02_write(0x01, MaxVol * 10);
				}else if(key_uu == 1) //高电压
				{
					MinVOl = MinVOl - 0.3;
					if(MinVOl < 0.000000000000001)
					{
						MinVOl=0;
					}
					x24c02_write(0x02, MinVOl * 10);
				}
				else if(key_uu == 2) //Uper
				{
					led_in &= 0x00;
					if(uled == 0)
						return;
					
					if(uled - 1 == lled)
					{
						if(uled - 2 >= 0)
						{
							uled = uled - 2;
						}
					}else
						uled --;
					x24c02_write(0x03, uled);
				}else
				{
					led_in &= 0x00;
					if(lled == 0)
						return;
					
					if(lled - 1 == uled)
					{
						if(lled - 2 >= 0)
						{
							lled = lled - 2;
						}
					}else
						lled --;
					x24c02_write(0x04, lled);
				}
			}
  		break;
		}
  }
}

void work_Proc()
{
	uint16_t adc2 = 0;
	HAL_ADC_Start(&hadc2);
	adc2 = HAL_ADC_GetValue(&hadc2);
	ao2 = adc2*3.3f/4096;
	
	if(ao2 < MinVOl)
	{
		if(first != 1)
		{
			uwTick_Led_Set_Point = uwTick;
		}
		first = 1;
		state = 2;
		
	}else if(ao2 > MaxVol)
	{
		if(first != 1)
		{
			uwTick_Led_Set_Point = uwTick;
		}
		first = 1;
		state = 1;
	}else
	{
		led_in &= 0x00;	
		first = 0;
		state = 0;
	}
	
}

void led_Proc()
{
	if((uwTick - uwTick_Led_Set_Point) > 200)
	{
		if((uwTick - uwTick_Led_Set_Point1) < 200)
			return;
		uwTick_Led_Set_Point1 = uwTick;
		
		if(state == 2)
			led_in ^= LED_Arr[lled];
		else if(state == 1)
			led_in ^= LED_Arr[uled];
		else if(state == 0)
			led_in &= 0x00;	
	}
	
	led_dis(led_in);
	
	/*if((uwTick -  uwTick_Led_Set_Point) < 200)
		return;
	
	uwTick_Led_Set_Point = uwTick;


	if(ao2 < MinVOl)
	{
		uwTick_Led_Set_Point = uwTick;
		led_in ^= LED_Arr[lled];
		state = 2;
		
	}else if(ao2 > MaxVol)
	{
		led_in ^= LED_Arr[uled];
		state = 1;
	}else
	{
		led_in &= 0x00;	
		state = 0;
	}
	led_dis(led_in);*/
	
}


void lcd_Proc()
{
	if(!sta_lcd)
	{
		sprintf((char*)ptr,"      Main");
		LCD_DisplayStringLine(Line0,(u8*)ptr);
		sprintf((char*)ptr,"   Vole:%.2f", ao2);
		LCD_DisplayStringLine(Line3,(u8*)ptr);

		if(state == 0)
		{
			sprintf((char*)ptr,"      Normal");
			LCD_DisplayStringLine(Line4,(u8*)ptr);
		}else if(state == 1)
		{
			sprintf((char*)ptr,"      Upper ");
			LCD_DisplayStringLine(Line4,(u8*)ptr);
		}else if(state == 2)
		{
			sprintf((char*)ptr,"      Lower ");
			LCD_DisplayStringLine(Line4,(u8*)ptr);
		}
	}else{
		sprintf((char*)ptr,"      Setting");
		LCD_DisplayStringLine(Line0,(u8*)ptr);
		if(key_uu == 0)
		{
			LCD_SetBackColor (Green);
			sprintf((char*)ptr,"  Max Volt:%.1f", MaxVol);
			LCD_DisplayStringLine(Line4,(u8*)ptr);
			LCD_SetBackColor (Black);
		}else 
		{
			sprintf((char*)ptr,"  Max Volt:%.1f", MaxVol);
			LCD_DisplayStringLine(Line4,(u8*)ptr);
		}
		if(key_uu == 1)
		{
			LCD_SetBackColor (Green);
			sprintf((char*)ptr,"  Max Volt:%.1f", MinVOl);
			LCD_DisplayStringLine(Line5,(u8*)ptr);
			LCD_SetBackColor (Black);
		}else
		{
			sprintf((char*)ptr,"  Max Volt:%.1f", MinVOl);
			LCD_DisplayStringLine(Line5,(u8*)ptr);
		}
		if(key_uu == 2)
		{
			LCD_SetBackColor (Green);
			sprintf((char*)ptr,"  Upper:LD%d", uled);
			LCD_DisplayStringLine(Line6,(u8*)ptr);
			LCD_SetBackColor (Black);
		}
		else
		{
			sprintf((char*)ptr,"  Upper:LD%d", uled);
			LCD_DisplayStringLine(Line6,(u8*)ptr);
		}
		if(key_uu == 3)
		{
			LCD_SetBackColor (Green);
			sprintf((char*)ptr,"  Lpper:LD%d", lled);
			LCD_DisplayStringLine(Line7,(u8*)ptr);
			LCD_SetBackColor (Black);
		}else
		{
			sprintf((char*)ptr,"  Lpper:LD%d", lled);
			LCD_DisplayStringLine(Line7,(u8*)ptr);
		}
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
  /* USER CODE BEGIN 2 */
	LCD_Init();
	LCD_Clear(Black);
	led_dis(0x00);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	I2CInit();
	
	start1 = x24c02_read(0x00);
	
	x24c02_write(0x00, 0x33);
	
	if(start1 == 0x33)
	{
		MaxVol = x24c02_read(0x01);
		MaxVol = MaxVol / 10.0f;
		MinVOl = x24c02_read(0x02);
		MinVOl = MinVOl / 10.0f;
		uled = x24c02_read(0x03);
		lled = x24c02_read(0x04);
	}else
	{
		x24c02_write(0x01, MaxVol * 10);
		x24c02_write(0x02, MinVOl * 10);
		x24c02_write(0x03, uled);
		x24c02_write(0x04, lled);
	}
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		key_Proc();
		led_Proc();
		lcd_Proc();
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
