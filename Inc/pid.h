#ifndef __pid_H
#define __pid_H
#include "stm32f4xx_hal.h"
#include "usart.h"

#define FILTER_MAX		12           //滤波取样数量
#define SHELL_INTEGRAL_LIMIT			200         //外环积分限幅
#define CORE_INTEGRAL_LIMIT			200						//内环积分限幅
#define PID_OUT_LIMIT			500								//pid输出限幅

typedef struct{
	float set_angle;
	float shell_p;
	float shell_i;
	float shell_d;
	float core_p;
	float core_i;
	float core_d;
	float pitch;
	float row;
	float yaw;
}PID_Value;

typedef struct{
	float now_angle_error;               //当前角度误差
	float angle_error_integral;          //角度误差积分
	float now_acc_error;                 //当前角速度误差
	float old_acc_error;                 //上次角速度误差
	float old_angle_error;               //上次角度误差
	float acc_error_integral;            //角速度误差积分
	float acc_error_differential;        //角速度误差微分
}PID_runValue;

void PID_Reset(void);
void PID_Control(void);

extern PID_Value pid_value;
extern uint8_t pidSwitch;
#endif
