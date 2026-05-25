/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name        :  AS5600.c
 * Description      :  AS5600 driver base on stm32f407
 ******************************************************************************
 * @attention
 *
 * COPYRIGHT:    Copyright (c) 2025  2710614006@qq.com
 * DATE:         FEB 03rd, 2025
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "AS5600.h"

/* Define -------------------------------------------------------------------*/
#define _2PI       		2 * PI
#define FILTER_ALPHA 	0.2f

/* Private Variables --------------------------------------------------------*/
AS5600 Sensor0 = {0};
AS5600 Sensor1 = {0};

/* Code ---------------------------------------------------------------------*/
/********************************************************************************
	编码器初始化 
*********************************************************************************/
void AS5600_Init(void)
{
	Sensor0.motorNum = 0;		// motorNum 设置为 0
	Sensor1.motorNum = 1; 		// motorNum 设置为 1
}

/********************************************************************************
	读取编码器角度值 
*********************************************************************************/
void AS5600_RdRawAngle(AS5600 *pStru, I2C_HandleTypeDef *hi2c)
{
    uint8_t rbuff[2] = {0};
    
    pStru->result = AS5600_RdReg(hi2c, READ_RAW_ANGLE, rbuff, 2);
	unsigned char ret = AS5600_RdReg(hi2c, READ_RAW_ANGLE, rbuff, 2);
	if (ret == AS5600_ERROR)
	{
		HAL_I2C_DeInit(hi2c);
		HAL_I2C_Init(hi2c);
	}
	else
	{
        pStru->rawAngle = (rbuff[0]&0x0f)<<8| rbuff[1];				// 获得原始数据
		pStru->angle = (pStru->rawAngle * _2PI) / 4095.0f;			// 将原始角度转换为弧度值
		pStru->degAngle = (pStru->rawAngle * 360.0f) / 4095.0f;		// 将原始角度转换为角度值
	}

}

/********************************************************************************
	获取速度 
*********************************************************************************/
float32_t Get_M0_Velocity(AS5600 *pStru)
{
    static uint32_t lastTick = 0;
    uint32_t curTick = get_micros();
    uint32_t deltaT = delta_time_us(lastTick);

    // 初始化（同步时间戳和角度基准）
    if(lastTick == 0 || pStru->lastTs == 0)
	{
        lastTick = curTick;
        pStru->lastTs = curTick;
        pStru->lastAngle = pStru->angle;
        pStru->position = pStru->numOfCircle;
        pStru->lastTotalAngle = pStru->numOfCircle * _2PI + pStru->angle;
        return 0.0f;
    }

    // 时间差保护（最小50us，最大10ms）
    if(deltaT < 50 || deltaT > 10000)
	{
        return pStru->velocity;
    }

    // 计算当前总角度（带方向连续性）
    float32_t totalAngle = pStru->numOfCircle * _2PI + pStru->angle;
    float32_t deltaAngle = totalAngle - pStru->lastTotalAngle;

    // 处理±2pi跳变
    if(deltaAngle > PI)
	{
        deltaAngle -= _2PI;
    } 
    else if(deltaAngle < -PI)
	{
        deltaAngle += _2PI;
    }

    // 计算瞬时速度
    float32_t delta_t_s = deltaT * 1e-6f;
    float32_t velocity = deltaAngle / delta_t_s;

    // 低通滤波（截止频率100Hz）
    pStru->velocity = FILTER_ALPHA * velocity + (1 - FILTER_ALPHA) * pStru->velocity;

    // 更新历史基准
    pStru->lastTotalAngle = totalAngle;
    pStru->lastTs = curTick;
    lastTick = curTick;

    return pStru->velocity;
}

float32_t Get_M1_Velocity(AS5600 *pStru)
{
    static uint32_t lastTick = 0;
    uint32_t curTick = get_micros();
    uint32_t deltaT = delta_time_us(lastTick);

    // 初始化（同步时间戳和角度基准）
    if(lastTick == 0 || pStru->lastTs == 0)
	{
        lastTick = curTick;
        pStru->lastTs = curTick;
        pStru->lastAngle = pStru->angle;
        pStru->position = pStru->numOfCircle;
        pStru->lastTotalAngle = pStru->numOfCircle * _2PI + pStru->angle;
        return 0.0f;
    }

    // 时间差保护（最小50us，最大10ms）
    if(deltaT < 50 || deltaT > 10000)
	{
        return pStru->velocity;
    }

    // 计算当前总角度（带方向连续性）
    float32_t totalAngle = pStru->numOfCircle * _2PI + pStru->angle;
    float32_t deltaAngle = totalAngle - pStru->lastTotalAngle;

    // 处理±2pi跳变
    if(deltaAngle > PI)
	{
        deltaAngle -= _2PI;
    } 
    else if(deltaAngle < -PI)
	{
        deltaAngle += _2PI;
    }

    // 计算瞬时速度
    float32_t delta_t_s = deltaT * 1e-6f;
    float32_t velocity = deltaAngle / delta_t_s;

    // 低通滤波（截止频率100Hz）
    pStru->velocity = FILTER_ALPHA * velocity + (1 - FILTER_ALPHA) * pStru->velocity;

    // 更新历史基准
    pStru->lastTotalAngle = totalAngle;
    pStru->lastTs = curTick;
    lastTick = curTick;

    return pStru->velocity;
}

/********************************************************************************
	获取位置 
*********************************************************************************/
void M0_UpdatePosition(AS5600 *pStru, float32_t ZeroEAngle)
{
    static float32_t prevDeg = 0.0f;
  
    pStru->curDegAngle = normalizeDegAngle((pStru->rawAngle - ZeroEAngle) * 360.0f / 4096.0f);
    
    float delta = pStru->curDegAngle - prevDeg;
    if (fabsf(delta) > 180.0f)
	{	// 角度跳变超过180°时判定为跨圈
        if (delta > 0)
		{
            pStru->numOfCircles--;  // 顺时针跨圈（如360°→ 0°）
        } else
		{
            pStru->numOfCircles++;  // 逆时针跨圈（如0°→ 360°）
        }
    }
    prevDeg = pStru->curDegAngle;

    pStru->absDegAngle = pStru->curDegAngle + pStru->numOfCircles * 360.0f;
}

void M1_UpdatePosition(AS5600 *pStru, float32_t ZeroEAngle)
{
    static float32_t prevDeg = 0.0f;
  
    pStru->curDegAngle = normalizeDegAngle((pStru->rawAngle - ZeroEAngle) * 360.0f / 4096.0f);
    
    float delta = pStru->curDegAngle - prevDeg;
    if (fabsf(delta) > 180.0f)
	{	// 角度跳变超过180°时判定为跨圈
        if (delta > 0)
		{
            pStru->numOfCircles--;  // 顺时针跨圈（如360°→ 0°）
        } else
		{
            pStru->numOfCircles++;  // 逆时针跨圈（如0°→ 360°）
        }
    }
    prevDeg = pStru->curDegAngle;

    pStru->absDegAngle = pStru->curDegAngle + pStru->numOfCircles * 360.0f;
}


/********************************************************************************
	编码器通信函数
*********************************************************************************/
static uint8_t AS5600_RdReg(I2C_HandleTypeDef *hi2c, uint16_t regAdd, uint8_t *pData, uint16_t Size)
{
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Read(hi2c, AS5600_SLAVE_ADDRESS, regAdd, I2C_MEMADD_SIZE_8BIT, pData, Size, 100);
	
	if (status == HAL_OK)
		return AS5600_OK;
	else
		return AS5600_ERROR; 
}
 
uint8_t AS5600_WeReg(I2C_HandleTypeDef *hi2c, uint16_t regAdd, uint8_t *pData, uint16_t Size)
{
  HAL_StatusTypeDef status;
  status = HAL_I2C_Mem_Write(hi2c, AS5600_SLAVE_ADDRESS, regAdd, I2C_MEMADD_SIZE_8BIT, pData, Size, 1000);
  
	if (status == HAL_OK)
    return AS5600_OK;
  else
    return AS5600_ERROR; 
}

/********************************************************************************
	获取当前计数值（32位无符号整数）
*********************************************************************************/
uint32_t get_micros(void)
{
    return __HAL_TIM_GET_COUNTER(&htim2); // 读取TIM2->CNT寄存器
}

/********************************************************************************
	时间差计算（处理32位溢出）
*********************************************************************************/
uint32_t delta_time_us(uint32_t old_stamp)
{
    uint32_t new_stamp = __HAL_TIM_GET_COUNTER(&htim2);
    return (new_stamp - old_stamp) & 0xFFFFFFFF; // 利用32位无符号数自动溢出特性
}

/* End of this file */
