/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "stdbool.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "lcd.h"
#include "FOC.h"
#include "AS5600.h"
#include "Filter.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
struct MovingAverageFilter vol_avg_filter;
struct MovingAverageFilter temp1_avg_filter;
struct MovingAverageFilter temp2_avg_filter;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t Pre_d, Comp_d, Duty;
uint16_t TFT_ADC_value; 			// TFT屏幕的ADC转换值
float32_t TFT_Real_value; 			// TFT滑动变阻器实际电压值

uint16_t Voltage_ADC_value;			// 电源的ADC转换值
float32_t Voltage_Real_value; 		// 电源的实际电压值

uint16_t Temp1_ADC_value;			// 温度的ADC转换值
uint16_t Temp2_ADC_value;
float32_t Temp1_Vol_value;			// 温度的实际电压值
float32_t Temp2_Vol_value;
float32_t R1_ntc;					// NTC值
float32_t R2_ntc;
float32_t temperature1;				// 温度的实际值
float32_t temperature2;

uint8_t powerMode = 1;				// 电源供电模式
uint8_t S0_Angle = 0;				// 舵机角度
uint8_t S1_Angle = 0;

uint8_t rx_buffer[50];				// 串口缓存
uint8_t rx_len;						// 串口数据长度
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  Start();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint8_t	SPI1_WriteByte(uint8_t* data, uint16_t size)
{
//	return HAL_SPI_Transmit(&hspi1, data, size, 0xff);
	return HAL_SPI_Transmit(&hspi1, data, size, 0xff);
}

uint8_t count1 = 0;
uint8_t count2 = 1;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM4)
	{
		/* 计算TFT分压电阻电压值 */
		TFT_ADC_value = Read_ADC_Value(&hadc2, ADC_CHANNEL_0);  				// 调用函数触发ADC
		TFT_Real_value = TFT_ADC_value * (3.3f / 4096);  						// 计算实际电压
		Duty = TFT_Real_value * (100 / 3.3f);									// 计算占空比
		Comp_d = Pre_d * Duty / 100;
		
		/* 设置比较值 */
		if (__HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_1) != Comp_d)
		{
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, Comp_d);
		}
		
		ADC_dispose();
		static uint8_t i = 0;
		if (i == 9)
		{
			i = 0;
			/* 计算电源电压值 */
			Voltage_ADC_value = Read_ADC_Value(&hadc1, ADC_CHANNEL_1);			// 调用函数触发ADC
			Voltage_Real_value = Voltage_ADC_value  * (3.3f / 4096) * 11.0f;	// 计算实际电压
//	 		Voltage_Real_value = MovingAvg_Update(&vol_avg_filter, Voltage_Real_value);
			
			/* 计算热敏电阻值 */		
			Temp1_ADC_value = adcValue.value1;
			Temp2_ADC_value = adcValue.value2;
			Temp1_Vol_value = Temp1_ADC_value  * (3.3f / 4096);
			Temp2_Vol_value = Temp2_ADC_value  * (3.3f / 4096);
			R1_ntc = (3.3f * 3.0f / Temp1_Vol_value) - 1;
			R2_ntc = (3.3f * 3.0f / Temp2_Vol_value) - 1;
			temperature1 = R_ntc_to_temperature(R1_ntc, 0.0f, 60.0f, 1.0f);
			temperature2 = R_ntc_to_temperature(R2_ntc, 0.0f, 60.0f, 1.0f);
			temperature1 = MovingAvg_Update(&temp1_avg_filter, temperature1);
			temperature2 = MovingAvg_Update(&temp2_avg_filter, temperature2);
		}
		i++;
//		printf("%.3f, %.3f, %.3f, %.3f\n", TFT_Real_value, Voltage_Real_value, temperature1, temperature2);
	}
	if (htim->Instance == TIM6)
	{
		count1++;
		if (count1 == 6)
		{
			count1 = 0;
			LCD_Show_Vol_Num(Voltage_Real_value);
			LCD_Show_Servo_Angle(S0_Angle, S1_Angle);
			LCD_Show_Poewr_Num(Voltage_Real_value);
			LCD_Show_Temperature_Num(temperature1, temperature2);
		}

		LCD_Show_Motor_Mode(M0.mode, M1.mode);
		LCD_Show_Motor_Vel(Sensor0.velocity * 11.1f, Sensor1.velocity * 11.1f); // 11.1 = 0.185 * 60
		
		if (count1 == 4)
		{
			switch (count2)
			{
				case 1:	
				HAL_GPIO_WritePin(GPIOD, LEDA_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD, LEDB_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD, LEDC_Pin, GPIO_PIN_RESET);
				count2 = 2;
				break;
				case 2:	
				HAL_GPIO_WritePin(GPIOD, LEDA_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOD, LEDB_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD, LEDC_Pin, GPIO_PIN_RESET);
				count2 = 3;
				break;
				case 3:	
				HAL_GPIO_WritePin(GPIOD, LEDA_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD, LEDB_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOD, LEDC_Pin, GPIO_PIN_RESET);
				count2 = 4;
				break;
				case 4:	
				HAL_GPIO_WritePin(GPIOD, LEDA_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOD, LEDB_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOD, LEDC_Pin, GPIO_PIN_RESET);
				count2 = 5;
				break;
				case 5:	
				HAL_GPIO_WritePin(GPIOD, LEDA_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD, LEDB_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD, LEDC_Pin, GPIO_PIN_SET);
				count2 = 6;
				break;
				case 6:	
				HAL_GPIO_WritePin(GPIOD, LEDA_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD, LEDB_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOD, LEDC_Pin, GPIO_PIN_SET);
				count2 = 7;
				break;
				case 7:	
				HAL_GPIO_WritePin(GPIOD, LEDA_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOD, LEDB_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD, LEDC_Pin, GPIO_PIN_SET);

				count2 = 8;
				break;
				case 8:	
				HAL_GPIO_WritePin(GPIOD, LEDA_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOD, LEDB_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOD, LEDC_Pin, GPIO_PIN_SET);
				count2 = 1;
				break;
			}	
		}
	}
}

void Start(void)
{
	Motor_TIM18_Init();
	LCD_TIM4_Init();
	LCD_Init();
	Pre_d = __HAL_TIM_GET_AUTORELOAD(&htim4);
	LCD_Clear(BLACK);
	HAL_Delay(100);
	LCD_Show_O1();
	HAL_Delay(100);
	
	AS5600_Init();
	LCD_Show_O2();
	HAL_Delay(100);
	MovingAvg_Init(&vol_avg_filter, 5);
	MovingAvg_Init(&temp1_avg_filter, 15);
	MovingAvg_Init(&temp2_avg_filter, 15);
	LCD_Show_O3();
	HAL_Delay(100);
	HAL_ADC_Start_IT(&hadc1);
	LCD_Show_O4();
	HAL_Delay(100);

	HAL_ADC_Start_IT(&hadc2);
	LCD_Show_O5();
	HAL_Delay(100);
	FOC_Init();
	LCD_Show_O6();
	LCD_Show_O7();
	HAL_Delay(800);
	LCD_Show_BG();
	
	Other_TIM26_Init();
	LCD_Show_Poewr_Num(Voltage_Real_value);
	LCD_Show_Temperature_Num(temperature1, temperature2);
	LCD_Show_Vol_Num(Voltage_Real_value);
	LCD_Show_Motor_Mode(M0.mode, M1.mode);
	LCD_Show_Motor_Vel(Sensor0.velocity, Sensor1.velocity);
	LCD_Show_Servo_Angle(S0_Angle, S1_Angle);
	
	HAL_ADCEx_InjectedStart(&hadc1);
	HAL_ADC_Start_IT(&hadc1);
	HAL_ADCEx_InjectedStart(&hadc2);
	HAL_ADC_Start_IT(&hadc2);
	HAL_ADC_Start_DMA(&hadc3, (uint32_t *)adcData, 2);
	
	__HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOC);
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_BREAK);
	__HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_JEOC);
	__HAL_TIM_ENABLE_IT(&htim8, TIM_IT_BREAK);
}
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
