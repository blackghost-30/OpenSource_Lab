/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name        :  Motor.h
 * Description      :  Motor algorithm base on stm32f407
 ******************************************************************************
 * @attention
 *
 * COPYRIGHT:    Copyright (c) 2025  2710614006@qq.com
 * DATE:         Mar 23rd, 2025
 ******************************************************************************
 */
/* USER CODE END Header */

#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef _cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "usart.h"
#include "stm32f4xx_it.h"
#include "stdio.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "AS5600.h"
#include "ADC.h"
#include "Controller.h"
#include "Filter.h"
#include "FOC.h"

/* Private Variables ------------------------------------------------------------*/
#define ADC_MAX      			4096.0f			// 12位ADC最大值
#define V_REF 					3.3f   			// ADC参考电压3.3V
#define R_SHUNT          		0.01f  			// 10mΩ采样电阻
#define GAIN          			50.0f  			// TP181A1增益50
		
static float32_t adc_offset_1 = 0.0f;  			// U相零点偏移
static float32_t adc_offset_2 = 0.0f;  			// W相零点偏移
static float32_t adc_offset_3 = 0.0f;  			// U相零点偏移
static float32_t adc_offset_4 = 0.0f;  			// W相零点偏移
static float32_t adc_offset_5 = 0.0f;  			// M0零点偏移
static float32_t adc_offset_6 = 0.0f;  			// M1零点偏移

// 电机模式
typedef enum {
    MODE_OPEN = 0,      // 开环模式
    MODE_CURRENT = 1,   // 电流模式
    MODE_VELOCITY = 2,  // 速度模式
    MODE_POSITION = 3   // 位置模式
} MotorMode;

// 电机控制参数
typedef struct {
    MotorMode mode;
    union {
        float32_t Ope;  // 操作模式参数
        float32_t Cur;  // 电流模式参数
        float32_t Vel;  // 速度模式参数
        float32_t Pos;  // 位置模式参数
    } param;
} MotorControl;

// 电机参数
typedef struct __MOTORSTRUCT
{
	int 				motorNum;		//目标电机
	int 				dir;			// 设置电机方向
	int					polePairs;		// 电机极对数
}Motor;

// 电流环
typedef struct __CURRENTSTRUCT
{
	float32_t id;		//d轴电流反馈
	float32_t iq;		//q轴电流反馈
	float32_t Ud;		//d轴电压输出
	float32_t Uq;		//q轴电压输出

	float32_t iu;
	float32_t iv;
	float32_t iw;
	
	float32_t ialpha;
	float32_t ibeta;
}CURRENT_s;
//uint32_t adc_value_1 = 0;
//uint32_t adc_value_2 = 0;

//uint16_t aADCxINJConvertedData[4];			//注入采样数组uint

extern CURRENT_s M0_Curs;
extern CURRENT_s M1_Curs;
extern Motor M0_Motor;
extern Motor M1_Motor;
extern float32_t M0_Vel_T;
extern MotorControl M0;
extern MotorControl M1;
void Motor_Init(Motor *pStru, int motorNum, int dir, int polePairs);
void CurLoopInit(CURRENT_s *current);
void CalibrateCurrentOffset(void);

#ifdef _cplusplus
}
#endif

#endif 