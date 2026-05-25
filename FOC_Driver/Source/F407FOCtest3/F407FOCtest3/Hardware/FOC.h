/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name        :  FOC.h
 * Description      :  FOC algorithm base on stm32f407
 ******************************************************************************
 * @attention
 *
 * COPYRIGHT:    Copyright (c) 2025  2710614006@qq.com
 * DATE:         Mar 23rd, 2025
 ******************************************************************************
 */
/* USER CODE END Header */

#ifndef __FOC_H
#define __FOC_H

#ifdef _cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "usart.h"
#include "stdio.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "AS5600.h"
#include "ADC.h"
#include "Controller.h"
#include "Filter.h"
#include "Motor.h"

/* Private Variables ---------------------------------------------------------*/
typedef struct __SVPWMOUTPUT
{
	float32_t 			Tcmp1;
	float32_t 			Tcmp2;
	float32_t 			Tcmp3;
}SVPWM_Outputs;

extern float32_t zero;
extern float32_t M0_velocity;
extern float32_t M1_velocity;

/* Private function prototypes -----------------------------------------------*/
void FOC_Init(void);
float32_t normalizeAngle(float32_t angle);
float32_t normalizeDegAngle(float32_t angle);

float32_t M0_GetElectric_Angle(float32_t rawAngle);
float32_t M1_GetElectric_Angle(float32_t angle);
float32_t M1_GetElectric_Angle(float32_t angle);
float32_t M1_GetElectric_Angle(float32_t angle);

void Clarke_Transform(float32_t iu, float32_t iv, float32_t iw, float32_t* i_alpha, float32_t* i_beta);
void Park_Transform(float32_t ialpha, float32_t ibeta, float32_t eAngle, float32_t* id, float32_t* iq);

void M0_OpeLoop(float32_t Target);
void M1_OpeLoop(float32_t Target);
void M0_CurLoop(float32_t Target);
void M1_CurLoop(float32_t Target);
void M0_VelLoop(float32_t Target);
void M1_VelLoop(float32_t Target);
void M0_PosLoop(float32_t Target);
void M1_PosLoop(float32_t Target);

SVPWM_Outputs CalculateSVPWM(float32_t Uq, float32_t Ud, float32_t eAngle);
void SetSVPWM(float32_t motorNum, float32_t Uq, float32_t Ud, float32_t eAngle);
float32_t EXlogic(float32_t target, float32_t iq);

#ifdef _cplusplus
}
#endif

#endif 