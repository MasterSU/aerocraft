#ifndef __pid_H
#define __pid_H
#include "stm32f4xx_hal.h"
#include "usart.h"

#define FILTER_MAX		12           //�˲�ȡ������
#define SHELL_INTEGRAL_LIMIT			200         //�⻷�����޷�
#define CORE_INTEGRAL_LIMIT			200						//�ڻ������޷�
#define PID_OUT_LIMIT			500								//pid����޷�

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
	float now_angle_error;               //��ǰ�Ƕ����
	float angle_error_integral;          //�Ƕ�������
	float now_acc_error;                 //��ǰ���ٶ����
	float old_acc_error;                 //�ϴν��ٶ����
	float old_angle_error;               //�ϴνǶ����
	float acc_error_integral;            //���ٶ�������
	float acc_error_differential;        //���ٶ����΢��
}PID_runValue;

void PID_Reset(void);
void PID_Control(void);

extern PID_Value pid_value;
extern uint8_t pidSwitch;
#endif
