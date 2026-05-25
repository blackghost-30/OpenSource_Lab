/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name        :  AS5600.h
 * Description      :  AS5600 driver base on stm32f407
 ******************************************************************************
 * @attention
 *
 * COPYRIGHT:    Copyright (c) 2025  2710614006@qq.com
 * DATE:         FEB 03rd, 2025
 ******************************************************************************
 */
/* USER CODE END Header */

#ifndef __AS5600_H
#define __AS5600_H 

#ifdef _cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "usart.h"
#include "i2c.h"
#include <stdlib.h>
#include <stdio.h>
#include "stdbool.h"
#include "Motor.h"
#include "FOC.h"

/* Define -------------------------------------------------------------------*/
#define ANGLE_PARA_E                    131040
#define _2PI       						2 * PI
 
#define	AS5600_SLAVE_ADDRESS           0x36<<1
#define	READ_RAW_ANGLE                 0x0C
#define	READ_ANGLE                     0x0E

/* Private Variables --------------------------------------------------------*/
enum A5600_STATUS
{
	AS5600_OK = 0 ,
	AS5600_ERROR
};
 
typedef struct __AS5600STRUCT
{
	int			motorNum;
	uint16_t 	rawAngle;			// 原始数据：0-4095
	float32_t	degAngle;			// 角度：0-360
	int16_t  	numOfCircle;		// 圈数
	float32_t 	angle;				// 角度：0-2pi
	float32_t 	eAngle;				// 电角度
	bool 		result;       		// false:不使用	true:使用
	float32_t 	velocity;			// 速度
	float32_t	position;			// 位置
	float32_t 	lastAngle;			// 上一个角度
	uint32_t 	lastTs;				// 上一个时间
	float32_t 	lastTotalAngle; 	// 带方向连续性的总角度缓存
	float32_t   curDegAngle; 		// 当前单圈角度（0-360°）
    int32_t  	numOfCircles;		// 累计圈数（正数为顺时针，负数为逆时针）
    float32_t   absDegAngle;		// 绝对角度 = curDegAngle + numOfCircles*360
}AS5600;

/* Private function prototypes -----------------------------------------------*/ 
extern AS5600 Sensor0;
extern AS5600 Sensor1;

void Init_Motor_Num(AS5600 *pStru, int motorNum);
void AS5600_Init(void);

void AS5600_RdRawAngle(AS5600 *pStru, I2C_HandleTypeDef *hi2c);

void M0_UpdatePosition(AS5600 *pStru, float32_t ZeroEAngle);
void M1_UpdatePosition(AS5600 *pStru, float32_t ZeroEAngle);

float32_t Get_M0_Velocity(AS5600 *pStru);
float32_t Get_M1_Velocity(AS5600 *pStru);

static uint8_t AS5600_RdReg(I2C_HandleTypeDef *hi2c, uint16_t regAdd, uint8_t *pData, uint16_t Size);
uint8_t AS5600_WeReg(I2C_HandleTypeDef *hi2c, uint16_t regAdd, uint8_t *pData, uint16_t Size);

uint32_t get_micros(void);
uint32_t delta_time_us(uint32_t old_stamp);

#ifdef _cplusplus
}
#endif

#endif 