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

static __IO uint32_t Key_Tick,DKey_Tick;
//lcd
u8 sta_lcd;
u8 ptr[30];

u8 pwm_stat;
unsigned char rx,rx_buf[30],dex;

u8 tc[128];
u8 tcs = 0;

//led
u8 led_in;
u8 e2prom[5];//EEPROM存储数组

typedef struct
{
	u8 num;
	u8 price;
}CLData;

CLData vnbr = {0, 20}, cnbr = {0, 35};

typedef struct
{
	unsigned char id[5];
	unsigned char type[5];
	unsigned char year;
	unsigned char month;
	unsigned char day;
	unsigned char hour;
	unsigned char min;
	unsigned char sec;
	unsigned char isempty;
}Database;
Database Car_data[8];


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
			sta_lcd = !sta_lcd;
			LCD_Clear(Black);
			break;
		}
		case 2:
		{
			if(sta_lcd)
			{
				cnbr.price += 5;
				vnbr.price += 5;
			}
			break;
		}
		case 3:
		{
			if(sta_lcd)
			{
				if(vnbr.price - 5 >= 5)
				{
					cnbr.price -= 5;
					vnbr.price -= 5;
				}
			}
			break;
		}
		case 4:
		{
			pwm_stat = ~pwm_stat;
			if(pwm_stat)
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, 20);
			else
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, 0);
			break;
		}
	}
}

void lcd_Proc()
{
	if(!sta_lcd)
	{
		sprintf((char*)ptr,"       Data");
		LCD_DisplayStringLine(Line1,(u8*)ptr);
		sprintf((char*)ptr,"   CNBR:%d", cnbr.num);
		LCD_DisplayStringLine(Line3,(u8*)ptr);
		sprintf((char*)ptr,"   VNBR:%d", vnbr.num);
		LCD_DisplayStringLine(Line5,(u8*)ptr);
		sprintf((char*)ptr,"   IDLE:%d", 8 - cnbr.num - vnbr.num);
		LCD_DisplayStringLine(Line7,(u8*)ptr);
		
	}else{
		sprintf((char*)ptr,"       Para");
		LCD_DisplayStringLine(Line1,(u8*)ptr);
		sprintf((char*)ptr,"   CNBR:%4.2f", cnbr.price * 0.1f);
		LCD_DisplayStringLine(Line3,(u8*)ptr);
		sprintf((char*)ptr,"   VNBR:%4.2f", vnbr.price * 0.1f);
		LCD_DisplayStringLine(Line5,(u8*)ptr);
	}
}

void led_Proc()
{
	
	if((8 - cnbr.num - vnbr.num) > 0)
	{
		led_in |= 0x01;
	}else
	{
		led_in &= ~(0x01);
	}
	
	if(pwm_stat)
	{
		led_in |= 0x02;
	}else
	{
		led_in &= ~(0x02);
	}
	
	led_dis(led_in);
}

unsigned char tx2[21];
unsigned char isRxCplt()
{
	if(dex!=22)
		return 0;
	
	if(((rx_buf[0]=='C')||(rx_buf[0]=='V'))&&(rx_buf[1]=='N')&&(rx_buf[2]=='B')&&(rx_buf[3]=='R')&&(rx_buf[4]==':')&&(rx_buf[9]==':'))//确保车辆类型和标点符号无误
	{
		unsigned char i;
		for(i=10;i<22;i++)
		{
			if((rx_buf[i]>'9')&&(rx_buf[i]<'0'))
				return 0;
		}
		return 1;
	}
	else
	{
		return 0;
	}
		
}

unsigned char tx[30];

void send_error()
{
	sprintf((char*)tx,"Error\r\n");
	HAL_UART_Transmit(&huart1,tx,sizeof(tx),50);
}

unsigned char isExis(unsigned char* str)
{
	unsigned char i;
	
	for(i=0;i<8;i++)
	{
		if(strcmp((char*)str,(char*)Car_data[i].id)==0)
			return i;
	}
	return 0xFF;
}

void Str_Tran(unsigned char* d_str,unsigned char* str,unsigned char	num,unsigned char lenth)
{
	unsigned char i;
	for(i=0;i<lenth;i++)
		d_str[i]=str[num+i];
	d_str[lenth]='\0';
}

unsigned char isEmpty()
{
	unsigned char i;
	
	for(i=0;i<8;i++)
	{
		if(Car_data[i].isempty==0)
			return i;
	}
	return 0xFF;
}

void Uart_Proc()
{
	
	static __IO uint32_t Uart_Tick;
	
	if(uwTick-Uart_Tick<50)
		return;
	Uart_Tick=uwTick;
	
	if(isRxCplt())
	{
		unsigned char car_id[5],car_type[5],year,month,day,hour,min,sec;
		
		year=(rx_buf[10]-'0')*10+(rx_buf[11]-'0');
		month=(rx_buf[12]-'0')*10+(rx_buf[13]-'0');
		day=(rx_buf[14]-'0')*10+(rx_buf[15]-'0');
		hour=(rx_buf[16]-'0')*10+(rx_buf[17]-'0');
		min=(rx_buf[18]-'0')*10+(rx_buf[19]-'0');
		sec=(rx_buf[20]-'0')*10+(rx_buf[21]-'0');
		
		if((month>12)||(day>31)||(hour>23)||(min>59)||(sec>59))
		{
			send_error();
			return;
		}
		
		Str_Tran(car_id,rx_buf,5,4);
		Str_Tran(car_type,rx_buf,0,4);
		
		if(isExis(car_id)==0xFF)
		{
			unsigned char in_locate=isEmpty();
			if(in_locate==0xFF)
			{
				send_error();
				return;
			}
			
			Str_Tran(Car_data[in_locate].type,car_type,0,4);
			Str_Tran(Car_data[in_locate].id,car_id,0,4);
			
			Car_data[in_locate].year=year;
			Car_data[in_locate].month=month;
			Car_data[in_locate].day=day;
			Car_data[in_locate].hour=hour;
			Car_data[in_locate].min=min;
			Car_data[in_locate].sec=sec;
			Car_data[in_locate].isempty=1;
			
			if(Car_data[in_locate].type[0]=='C')
				cnbr.num++;
			else if(Car_data[in_locate].type[0]=='V')
				vnbr.num++;
		}else
		{
			unsigned char out_locate=isExis(car_id);
			signed int time;
			
			if(strcmp((char*)car_type,(char*)Car_data[out_locate].type)!=0)
			{
				send_error();
				return;
			}
			
			time=(year-Car_data[out_locate].year)*365*24*60*60+(month-Car_data[out_locate].month)*31*24*60*60+(day-Car_data[out_locate].day)*24*60*60
			+(hour-Car_data[out_locate].hour)*60*60+(min-Car_data[out_locate].min)*60+(sec-Car_data[out_locate].sec);
			
			if(time<0)
			{
				send_error();
				return;
			}
			
			time=(time+3599)/3600;
			
			sprintf((char*)tx,"%s:%s:%d:%.2f\r\n",Car_data[out_locate].type,Car_data[out_locate].id,time,(Car_data[out_locate].type[0]=='C'?time*cnbr.price * 1.0f : time*vnbr.price * 1.0f));//计算停车费用，也可以用if语句来写
			HAL_UART_Transmit(&huart1,tx,sizeof(tx),50);
			
			if(Car_data[out_locate].type[0]=='C')
				cnbr.num--;
			else if(Car_data[out_locate].type[0]=='V')
				vnbr.num--;
			
			memset(&Car_data[out_locate],0,sizeof(Car_data[out_locate]));
		}
	}
	
	memset(&rx_buf,0,sizeof(rx_buf));
	dex=0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	
	rx_buf[dex++]=rx;
	
	HAL_UART_Receive_IT(&huart1, &rx,1);
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

	LCD_Init();
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	LCD_Clear(Black);
	led_dis(0);
	HAL_UART_Receive_IT(&huart1,&rx,1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
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
