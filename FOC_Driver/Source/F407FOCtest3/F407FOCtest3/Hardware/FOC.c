/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name        :  FOC.c
 * Description      :  FOC algorithm base on stm32f407
 ******************************************************************************
 * @attention
 *
 * COPYRIGHT:    Copyright (c) 2025  2710614006@qq.com
 * DATE:         Mar 23rd, 2025
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FOC.h"

/* Define -------------------------------------------------------------------*/
#define _2PI       		2 * PI
#define _3PI_2     		4.71238898f
#define _1_SQRT3 		0.57735027f
#define _2_SQRT3   		1.15470054f

#define M0_PolePairs	7
#define M1_PolePairs	7
#define M0_Dir			1
#define M1_Dir			-1
#define Udc				12.0f

/* Private Variables --------------------------------------------------------*/
PIDController M0_iq_pid;
PIDController M1_iq_pid;
PIDController M0_vel_pid;
PIDController M1_vel_pid;
PIDController M0_pos_pid;
PIDController M1_pos_pid;

struct MovingAverageFilter iq0_avg_filter;
struct MovingAverageFilter iq1_avg_filter;
struct MovingAverageFilter vel0_avg_filter;
struct MovingAverageFilter vel1_avg_filter;
struct MovingAverageFilter pos0_avg_filter;
struct MovingAverageFilter pos1_avg_filter;

float32_t M0_ZeroEAngle = 0.0f;
float32_t M1_ZeroEAngle = 0.0f;
float32_t zero = 0.0f;

/* Code -------------------------------------------------------------------*/
/********************************************************************************
	FOC初始化 
*********************************************************************************/
void FOC_Init(void)
{
	Motor_Init(&M0_Motor, 0, M0_Dir, M0_PolePairs); 		// motorNum 设置为 0
	Motor_Init(&M1_Motor, 1, M1_Dir, M1_PolePairs);			// motorNum 设置为 1
	
	CurLoopInit(&M0_Curs);
	CurLoopInit(&M1_Curs);
	
	PID_init(&M0_iq_pid, 0.8f, 2.0f, 0.0f, -5.0f, 5.0f, -3.0f, 3.0f);		/* Kp  Ki  Kd  outputMin  outputMax  integralMin  integralMax */
	PID_init(&M1_iq_pid, 0.8f, 2.0f, 0.0f, -5.0f, 5.0f, -3.0f, 3.0f);		/* Kp  Ki  Kd  outputMin  outputMax  integralMin  integralMax */
	
	PID_init(&M0_vel_pid, 5.0f, 0.6f, 0.0f, -1.0f, 1.0f, -3.0f, 3.0f);		/* Kp  Ki  Kd  outputMin  outputMax  integralMin  integralMax */
	PID_init(&M1_vel_pid, 5.0f, 0.6f, 0.0f, -1.0f, 1.0f, -3.0f, 3.0f);		/* Kp  Ki  Kd  outputMin  outputMax  integralMin  integralMax */
	
	PID_init(&M0_pos_pid, 1.5f, 0.3f, 0.05f, -300.0f, 300.0f, -2.0f, 2.0f);	/* Kp  Ki  Kd  outputMin  outputMax  integralMin  integralMax */
	PID_init(&M1_pos_pid, 1.5f, 0.3f, 0.05f, -300.0f, 300.0f, -2.0f, 2.0f);	/* Kp  Ki  Kd  outputMin  outputMax  integralMin  integralMax */

	HAL_GPIO_WritePin(GPIOE, MOTOR_EN1_Pin, GPIO_PIN_SET); // 电机启动引脚
	CalibrateCurrentOffset();

	SetSVPWM(0, 0.0f, 4.0f, 0.0f);
	SetSVPWM(1, 0.0f, -4.0f, 0.0f);

	HAL_Delay(1000);
	AS5600_RdRawAngle(&Sensor0, &hi2c1);
	AS5600_RdRawAngle(&Sensor1, &hi2c2);
	M0_ZeroEAngle = Sensor0.rawAngle;
	M1_ZeroEAngle = Sensor1.rawAngle;
	SetSVPWM(0, 0.0f, 0.0f, 0.0f);
	SetSVPWM(1, 0.0f, 0.0f, 0.0f);
//	printf("%.3f, %.3f\n",M0_ZeroEAngle, M1_ZeroEAngle);

	MovingAvg_Init(&iq0_avg_filter, 3);
	MovingAvg_Init(&iq1_avg_filter, 3);
//	MovingAvg_Init(&vel0_avg_filter, 20);
//	MovingAvg_Init(&vel1_avg_filter, 20);
//	MovingAvg_Init(&pos0_avg_filter, 30);
}

/********************************************************************************
	将角度归化到 0 到 2PI
*********************************************************************************/
float32_t normalizeAngle(float32_t angle)
{
	angle = fmodf(angle, 2 * PI);  // 取模运算
	return angle < 0 ? angle + 2 * PI : angle; // 处理负角度
}

/********************************************************************************
	将角度归化到 0 到 360
*********************************************************************************/
float32_t normalizeDegAngle(float32_t angle)
{
	angle = fmodf(angle, 360);  // 取模运算
	return angle < 0 ? angle + 360 : angle; // 处理负角度
}

/********************************************************************************
	获取电角度
*********************************************************************************/
float32_t M0_GetElectric_Angle(float32_t rawAngle)
{
	float32_t eAngle;
	rawAngle = rawAngle - M0_ZeroEAngle;
	eAngle = normalizeAngle(rawAngle * _2PI / 4096 * M0_PolePairs * M0_Dir);
	return eAngle;
}

float32_t M1_GetElectric_Angle(float32_t rawAngle)
{
	float32_t eAngle;
	rawAngle = rawAngle - M1_ZeroEAngle;
	eAngle = normalizeAngle(rawAngle * _2PI / 4096 * M1_PolePairs * M1_Dir);
//	printf("%.3f\n", angle);	
	return eAngle;
}

/********************************************************************************
	 Clarke变换（3相→2相）
*********************************************************************************/
void Clarke_Transform(float32_t iu, float32_t iv, float32_t iw, float32_t* i_alpha, float32_t* i_beta)
{
    /* 采用等幅值变换
     * 变换矩阵：
     * [ α ] = [     1             0     ] [ u ]
     * [ β ] = [ -1/sqrt(3)   -2/sqrt(3) ] [ V ]
     */
    *i_alpha = iu;
	*i_beta  = -(iu + 2 * iw) * _1_SQRT3;
}

/********************************************************************************
	 Park变换（静止坐标系→旋转坐标系）
*********************************************************************************/
void Park_Transform(float32_t ialpha, float32_t ibeta, float32_t eAngle, float32_t* id, float32_t* iq)
{
	float32_t sinTheta = arm_sin_f32(eAngle);
	float32_t cosTheta = arm_cos_f32(eAngle);
    /* 变换矩阵：
     * [ Id ] = [ cosθ    sinθ ] [ α ]
     * [ Iq ] = [ sinθ   -cosθ ] [ β ]
     */
	*id = ialpha * cosTheta + ibeta * sinTheta;
    *iq = ialpha * (-sinTheta) + ibeta * cosTheta;
}

/********************************************************************************
	 FOC开环（电压闭环）
*********************************************************************************/
void M0_OpeLoop(float32_t Target)
{
	float32_t eAngle;
	eAngle = M0_GetElectric_Angle(Sensor0.rawAngle);
	SetSVPWM(0, Target, 0.0f, eAngle);
}

void M1_OpeLoop(float32_t Target)
{
	float32_t eAngle;
	eAngle = M1_GetElectric_Angle(Sensor1.rawAngle);
	SetSVPWM(1, Target, 0.0f, eAngle);
}

/********************************************************************************
	 FOC电流闭环
*********************************************************************************/
void M0_CurLoop(float32_t Target)
{
	float32_t eAngle;
	eAngle = M0_GetElectric_Angle(Sensor0.rawAngle);
    
	Clarke_Transform(M0_Curs.iu, M0_Curs.iv, M0_Curs.iw, &M0_Curs.ialpha, &M0_Curs.ibeta);
// 	printf("%.3f, %.3f, %.3f\n", M0_Curs.iu, M0_Curs.iv, M0_Curs.iw);
 	Park_Transform(M0_Curs.ialpha, M0_Curs.ibeta, eAngle, &M0_Curs.id, &M0_Curs.iq);
//	M0_Curs.iq = EXlogic(Target, M0_Curs.iq);
//	printf("%.3f, %.3f, %.3f, %.3f, %.3f\n", M0_Curs.ialpha, M0_Curs.ibeta, M0_Curs.id, M0_Curs.iq, zero);
//	printf("%.3f, %.3f, %.3f, %.3f\n", M0_Curs.ialpha, Target, M0_Curs.iq, zero);
//	printf("%.3f, %.3f, %.3f\n", Target, M0_Curs.iq, zero);

	M0_Curs.Uq = PID(&M0_iq_pid, Target, M0_Curs.iq);
	SetSVPWM(M0_Motor.motorNum, M0_Curs.Uq, 0.0f, eAngle);
}

void M1_CurLoop(float32_t Target)
{
	float32_t eAngle;
	eAngle = M1_GetElectric_Angle(Sensor1.rawAngle);
    
	Clarke_Transform(M1_Curs.iu, M1_Curs.iv, M1_Curs.iw, &M1_Curs.ialpha, &M1_Curs.ibeta);
//	printf("%.3f, %.3f, %.3f\n", M1_Curs.iu, M1_Curs.iv, M1_Curs.iw);

 	Park_Transform(M1_Curs.ialpha, M1_Curs.ibeta, eAngle, &M1_Curs.id, &M1_Curs.iq);
//	M1_Curs.iq = EXlogic(Target, M1_Curs.iq);
//	M1_Curs.iq = MovingAvg_Update(&iq1_avg_filter, M1_Curs.iq);
//	printf("%.3f, %.3f, %.3f, %.3f, %.3f\n", M1_Curs.ialpha, M1_Curs.ibeta, M1_Curs.id, M1_Curs.iq, zero);
//	printf("%.3f, %.3f, %.3f, %.3f\n", M1_Curs.ialpha, Target, M1_Curs.iq, zero);
	
	M1_Curs.Uq = PID(&M1_iq_pid, Target, M1_Curs.iq);
	SetSVPWM(M1_Motor.motorNum, M1_Curs.Uq, 0.0f, eAngle);
}

/********************************************************************************
	 FOC速度闭环
*********************************************************************************/
void M0_VelLoop(float32_t Target)
{
	float32_t velocity;
	velocity = PID(&M0_vel_pid, Target / 60.0f, Sensor0.velocity * 0.185f);
//	printf("%.3f, %.3f\n", Target, Sensor0.velocity * 0.185f * 60.0f);
	M0_CurLoop(velocity);
}

void M1_VelLoop(float32_t Target)
{
	float32_t velocity;
	velocity = PID(&M1_vel_pid, Target / 60.0f, Sensor1.velocity * 0.185f);
//	printf("%.3f, %.3f\n", Target, Sensor1.velocity * 0.185f * 60.0f);

	M1_CurLoop(velocity);
}

/********************************************************************************
	 FOC位置闭环
*********************************************************************************/
void M0_PosLoop(float32_t Target)
{
    // 更新当前绝对角度
    M0_UpdatePosition(&Sensor0, M0_ZeroEAngle);
    
    // 计算绝对误差
    float32_t error = Target - Sensor0.absDegAngle;  
    float32_t position = PID(&M0_pos_pid, error, 0.0f);
    M0_VelLoop(position);
}

void M1_PosLoop(float32_t Target)
{
    // 更新当前绝对角度
    M1_UpdatePosition(&Sensor1, M1_ZeroEAngle);
    
    // 计算绝对误差
    float32_t error = Target - Sensor1.absDegAngle;
    float32_t position = PID(&M1_pos_pid, error, 0.0f);
//	printf("%.3f, %.3f\n", position, error);
    M1_VelLoop(position);
}

/********************************************************************************
	 计算SVPWM
*********************************************************************************/
SVPWM_Outputs CalculateSVPWM(float32_t Uq, float32_t Ud, float32_t eAngle)
{
	uint8_t sector;
	const float32_t Ts = 1.0f;
	const float32_t	TsDivUdc = Ts / Udc;
	float32_t sinAngle, cosAngle;
	float32_t Ualpha, Ubeta;
	float32_t Tcmp1, Tcmp2, Tcmp3, Tx, Ty, f_temp, Ta, Tb, Tc;
	SVPWM_Outputs outputs;
	
	sector = 0;
	sinAngle = 0.0f;
	cosAngle = 0.0f;
	Tcmp1 = 0.0f;
	Tcmp2 = 0.0f;
	Tcmp3 = 0.0f;

	sinAngle = arm_sin_f32(eAngle);
	cosAngle = arm_cos_f32(eAngle);

	Ualpha = Ud * cosAngle - Uq * sinAngle;
	Ubeta = Ud * sinAngle + Uq * cosAngle;

	if (Ubeta > 0.0f) 
		sector = 1;
	if ((1.73205078f * Ualpha - Ubeta) / 2.0f > 0.0f) 
		sector += 2;
	if ((-1.73205078f * Ualpha - Ubeta) / 2.0f > 0.0f) 
		sector += 4; 

	switch (sector) 
	{
		case 1:
		Tx = (-1.5f * Ualpha + 0.866025388f * Ubeta) * TsDivUdc;
		Ty = (1.5f * Ualpha + 0.866025388f * Ubeta) * TsDivUdc;
		break;

		case 2:
		Tx = (1.5f * Ualpha + 0.866025388f * Ubeta) * TsDivUdc;
		Ty = -(1.73205078f * Ubeta * TsDivUdc);
		break;

		case 3:
		Tx = -((-1.5f * Ualpha + 0.866025388f * Ubeta) * TsDivUdc);
		Ty = 1.73205078f * Ubeta * TsDivUdc;
		break;

		case 4:
		Tx = -(1.73205078f * Ubeta * TsDivUdc);
		Ty = (-1.5f * Ualpha + 0.866025388f * Ubeta) * TsDivUdc;
		break;

		case 5:
		Tx = 1.73205078f * Ubeta * TsDivUdc;
		Ty = -((1.5f * Ualpha + 0.866025388f * Ubeta) * TsDivUdc);
		break;

		default:
		Tx = -((1.5f * Ualpha + 0.866025388f * Ubeta) * TsDivUdc);
		Ty = -((-1.5f * Ualpha + 0.866025388f * Ubeta) * TsDivUdc);
		break;
	}
	
	f_temp = Tx + Ty;
	if (f_temp > Ts)
	{
		Tx /= f_temp;
		Ty /= f_temp;
	}

	Ta = (Ts - (Tx + Ty)) / 4.0f;
	Tb = Tx / 2.0f + Ta;
	Tc = Ty / 2.0f + Tb;
	switch (sector) {
		case 1:
			Tcmp1 = Tb;
			Tcmp2 = Ta;
			Tcmp3 = Tc;
			break;
			
		case 2:
			Tcmp1 = Ta;
			Tcmp2 = Tc;
			Tcmp3 = Tb;
			break;
			
		case 3:
			Tcmp1 = Ta;
			Tcmp2 = Tb;
			Tcmp3 = Tc;
			break;
			
		case 4:
			Tcmp1 = Tc;
			Tcmp2 = Tb;
			Tcmp3 = Ta;
			break;
			
		case 5:
			Tcmp1 = Tc;
			Tcmp2 = Ta;
			Tcmp3 = Tb;
			break;
			
		case 6:
			Tcmp1 = Tb;
			Tcmp2 = Tc;
			Tcmp3 = Ta;
			break;
	}
	outputs = (SVPWM_Outputs){Tcmp1 * 5600, Tcmp2 * 5600, Tcmp3 * 5600};	// 中央对齐模式，15kHZ
	return outputs;
}

/********************************************************************************
	 设置TIM比较寄存器
*********************************************************************************/
void SetSVPWM(float32_t motorNum, float32_t Uq, float32_t Ud, float32_t eAngle)
{
	if (motorNum == 0)
	{
		SVPWM_Outputs M0_outputs = CalculateSVPWM(Uq, Ud, eAngle);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, M0_outputs.Tcmp1);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, M0_outputs.Tcmp2);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, M0_outputs.Tcmp3);
	}
	if (motorNum == 1)
	{
		SVPWM_Outputs M1_outputs = CalculateSVPWM(Uq, Ud, eAngle);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, M1_outputs.Tcmp1);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, M1_outputs.Tcmp2);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, M1_outputs.Tcmp3);
	}
}

/********************************************************************************
	 iq额外逻辑
*********************************************************************************/
float32_t EXlogic(float32_t target, float32_t iq)
{
	if (target > 0)
	{
		if (iq < 0)	{iq = -iq;}
	}
	if (target < 0)
	{
		if (iq > 0)	{iq = -iq;}
	}
	return iq;
}
