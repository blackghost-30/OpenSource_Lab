/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name        :  Motor.c
 * Description      :  Motor algorithm base on stm32f407
 ******************************************************************************
 * @attention
 *
 * COPYRIGHT:    Copyright (c) 2025  2710614006@qq.com
 * DATE:         Mar 23rd, 2025
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "Motor.h"

/* Define ------------------------------------------------------------*/
#define _12Div4800	0.0025		// 12V电压，arr4800

/* Private Variables ------------------------------------------------------------*/
Motor M0_Motor;
Motor M1_Motor;

CURRENT_s M0_Curs;
CURRENT_s M1_Curs;

// 声明全局控制变量
MotorControl M0 = {
    .mode = MODE_OPEN,
    .param = { .Ope = 0.0f }	// M0联合体初始化
};

MotorControl M1 = {
    .mode = MODE_OPEN,
    .param = { .Ope = 0.0f }	// M1联合体初始化
};

/* Code ---------------------------------------------------------*/
/********************************************************************************
	 电机初始化
*********************************************************************************/
void Motor_Init(Motor *pStru, int motorNum, int dir, int polePairs)
{
	pStru -> motorNum = motorNum;
	pStru -> dir = dir;
	pStru -> polePairs = polePairs;
}

/********************************************************************************
	 电流初始化
*********************************************************************************/
void CurLoopInit(CURRENT_s *pStru)
{
	pStru -> id = 0.0f;
	pStru -> iq = 0.0f;
	pStru -> Ud = 0.0f;
	pStru -> Uq = 0.0f;
	pStru -> iu = 0.0f;
	pStru -> iv = 0.0f;
	pStru -> iw = 0.0f;
	pStru -> ialpha = 0.0f;
	pStru -> ibeta = 0.0f;
}

/********************************************************************************
	 adc校准（在电机静止时调用）
*********************************************************************************/
void CalibrateCurrentOffset(void)
{
	// 多次采样取平均以提高精度
	const uint8_t samples = 100;
	adc_offset_1 = 0;
	adc_offset_2 = 0;
	adc_offset_3 = 0;
	adc_offset_4 = 0;
//	adc_offset_5 = 0;
//	adc_offset_6 = 0;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_Start(&hadc2);
	for (int i = 0; i < samples; i++)
	{
		adc_offset_1 += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
		adc_offset_2 += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
		adc_offset_3 += HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
		adc_offset_4 += HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
//		adc_offset_5 += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
//		adc_offset_6 += HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3);
		HAL_Delay(1); // 适当延时确保采样间隔
	}
	adc_offset_1 /= samples;
	adc_offset_2 /= samples;
	adc_offset_3 /= samples;
	adc_offset_4 /= samples;
//	adc_offset_5 /= samples;
//	adc_offset_6 /= samples;
}

/********************************************************************************
	 读取ADC注入组的值
*********************************************************************************/
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
		/*** 获取ADC原始值 ***/	
		uint32_t adc_value_1 = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
		uint32_t adc_value_2 = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
//		uint32_t adc_value_3 = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);

		/*** 转换为实际电压（考虑偏置） ***/
		float32_t u_1 = V_REF * ((float32_t)(adc_value_1 - adc_offset_1) / ADC_MAX);
		float32_t u_2 = V_REF * ((float32_t)(adc_value_2 - adc_offset_2) / ADC_MAX);
//		float32_t u_3 = V_REF * ((float32_t)(adc_value_3 - adc_offset_5) / ADC_MAX);

		/*** 计算相电流（I = V/(R*G)） ***/
		M0_Curs.iw = u_1 / (R_SHUNT * GAIN);
		M0_Curs.iu = u_2 / (R_SHUNT * GAIN);
		M0_Curs.iv =  -(M0_Curs.iu + M0_Curs.iw);
	  
		AS5600_RdRawAngle(&Sensor0, &hi2c1);
		Get_M0_Velocity(&Sensor0);

		if (M0.mode == 0)
		{
			M0_OpeLoop(M0.param.Ope);
		}
		else if (M0.mode == 1)
		{
			M0_CurLoop(M0.param.Cur);
		}
		else if (M0.mode == 2)
		{
			M0_VelLoop(M0.param.Vel); // 单 MAX 550rmp | 双 MAX 450rmp
		}
		else if (M0.mode == 3)
		{
			M0_PosLoop(M0.param.Pos);
		}
	}
  if (hadc->Instance == ADC2)
  {
		/*** 获取ADC原始值 ***/	
		uint32_t adc_value_1 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
		uint32_t adc_value_2 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
//		uint32_t adc_value_3 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3);

		/*** 转换为实际电压（考虑偏置） ***/
		float32_t u_1 = V_REF * ((float32_t)(adc_value_1 - adc_offset_3) / ADC_MAX);
		float32_t u_2 = V_REF * ((float32_t)(adc_value_2 - adc_offset_4) / ADC_MAX);
//		float32_t u_3 = V_REF * ((float32_t)(adc_value_3 - adc_offset_6) / ADC_MAX);

		/*** 计算相电流（I = V/(R*G)） ***/
		M1_Curs.iw = u_1 / (R_SHUNT * GAIN);
		M1_Curs.iu = u_2 / (R_SHUNT * GAIN);
		M1_Curs.iv =  -(M1_Curs.iu + M1_Curs.iw);
	  
		AS5600_RdRawAngle(&Sensor1, &hi2c2);
		Get_M1_Velocity(&Sensor1);
		
		if (M1.mode == 0)
		{
			M1_OpeLoop(M1.param.Ope);
		}
		else if (M1.mode == 1)
		{
			M1_CurLoop(M1.param.Cur);
		}
		else if (M1.mode == 2)
		{
			M1_VelLoop(M1.param.Vel); // 单 MAX 550rmp | 双 MAX 450rmp
		}
		else if (M1.mode == 3)
		{
			M1_PosLoop(M1.param.Pos);
		}
    }
}
