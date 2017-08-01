#include "pid.h"
#include "main.h"
#include "tim.h"
//����̬�Ǵ���ĳ��ֵ��ʱ��ֹͣpid���ƹرյ����


uint8_t pidSwitch=0;                                              //pid����
PID_Value pid_value;                                              //pid�����ṹ��
PID_runValue pitch_runvalue,row_runvalue,yaw_runvalue;            //pid���б����ṹ��

//����pid���б����ṹ��
void PID_Reset()
{
	pitch_runvalue.acc_error_differential=0;
	pitch_runvalue.acc_error_integral=0;
	pitch_runvalue.angle_error_integral=0;
	pitch_runvalue.now_acc_error=0;
	pitch_runvalue.now_angle_error=0;
	pitch_runvalue.old_acc_error=0;
	row_runvalue.acc_error_differential=0;
	row_runvalue.acc_error_integral=0;
	row_runvalue.angle_error_integral=0;
	row_runvalue.now_acc_error=0;
	row_runvalue.now_angle_error=0;
	row_runvalue.old_acc_error=0;
	yaw_runvalue.acc_error_differential=0;
	yaw_runvalue.acc_error_integral=0;
	yaw_runvalue.angle_error_integral=0;
	yaw_runvalue.now_acc_error=0;
	yaw_runvalue.now_angle_error=0;
	yaw_runvalue.old_acc_error=0;
}

uint8_t fx_use=0,fy_use=0;
uint16_t speed1,speed2,speed3,speed4;
float filter_acc_x_e=0,filter_acc_y_e=0;
float old_filter_acc_x_e=0,old_filter_acc_y_e=0;
float shell_p_out=0,shell_i_out=0,shell_d_out=0,shell_out=0;      //�⻷pid��������������
float core_p_out=0,core_i_out=0,core_d_out=0;      //�ڻ�pid����������
float pitch_out=0,row_out=0,yaw_out=0;    //������̬��pid���
//pid
void PID_Control()
{
	if(!pidSwitch) 
	{
		set_motor_speed(1000,1000,1000,1000);
		return;
	}//���δ����pid�򷵻�

	//pitch 
	/*****************shell*******************/
	pitch_runvalue.now_angle_error = pid_value.pitch - sensor_value.ang_p;
	shell_p_out = pid_value.shell_p * pitch_runvalue.now_angle_error;
	if(control_speed>=100)  //���Ŵ���100�ٻ���
	{
		pitch_runvalue.angle_error_integral += pitch_runvalue.now_angle_error;
		if(pitch_runvalue.angle_error_integral > SHELL_INTEGRAL_LIMIT)      //�����޷�
			pitch_runvalue.angle_error_integral = SHELL_INTEGRAL_LIMIT; 
		else if(pitch_runvalue.angle_error_integral < -SHELL_INTEGRAL_LIMIT)     //�����޷�
			pitch_runvalue.angle_error_integral = -SHELL_INTEGRAL_LIMIT;
	}
	else   //С��100������
		pitch_runvalue.angle_error_integral = 0;
	shell_i_out = pitch_runvalue.angle_error_integral * pid_value.shell_i;
	shell_out = shell_p_out + shell_i_out;       //�⻷ֻ��pi
	/*****************core********************/
	if(!fy_use)
		pitch_runvalue.now_acc_error = shell_out - sensor_value.acc_y;
	else
		filter_acc_y_e = shell_out - filter_acc_y;
	core_p_out = pitch_runvalue.now_acc_error * pid_value.core_p;
	if(control_speed>=100)  //���Ŵ���5�ٻ���
	{
		pitch_runvalue.acc_error_integral += pitch_runvalue.now_acc_error;
		if(pitch_runvalue.acc_error_integral > CORE_INTEGRAL_LIMIT)         //�����޷�
			pitch_runvalue.acc_error_integral = CORE_INTEGRAL_LIMIT; 
		else if(pitch_runvalue.acc_error_integral < -CORE_INTEGRAL_LIMIT)    //�����޷�
			pitch_runvalue.acc_error_integral = -CORE_INTEGRAL_LIMIT;
	}
	else    //С��5������
		pitch_runvalue.acc_error_integral = 0;
	core_i_out = pitch_runvalue.acc_error_integral * pid_value.core_i;
	if(!fy_use)
		pitch_runvalue.acc_error_differential = pitch_runvalue.now_acc_error - pitch_runvalue.old_acc_error;      //�ڻ�΢��
	else
		pitch_runvalue.acc_error_differential = filter_acc_y_e - old_filter_acc_y_e; 
	core_d_out = pitch_runvalue.acc_error_differential * pid_value.core_d;
	pitch_out = core_p_out + core_i_out + core_d_out;     //�ڻ���pid
	if(pitch_out>PID_OUT_LIMIT) pitch_out=PID_OUT_LIMIT;
	if(pitch_out<-PID_OUT_LIMIT) pitch_out=-PID_OUT_LIMIT;
	
	//row
	/*****************shell*******************/
	row_runvalue.now_angle_error = pid_value.row - sensor_value.ang_p;
	shell_p_out = pid_value.shell_p * row_runvalue.now_angle_error;
	if(control_speed>=100)
	{
		row_runvalue.angle_error_integral += row_runvalue.now_angle_error;
		if(row_runvalue.angle_error_integral > SHELL_INTEGRAL_LIMIT)
			row_runvalue.angle_error_integral = SHELL_INTEGRAL_LIMIT;
		else if(row_runvalue.angle_error_integral < -SHELL_INTEGRAL_LIMIT)
			row_runvalue.angle_error_integral = -SHELL_INTEGRAL_LIMIT;
	}
	else
		row_runvalue.angle_error_integral = 0;
	shell_i_out = row_runvalue.angle_error_integral * pid_value.shell_i;
	shell_out = shell_p_out + shell_i_out;
	/*****************core********************/
	if(fx_use)    //����ֵδ�ռ���
		row_runvalue.now_acc_error = shell_out - sensor_value.acc_x;
	else  //�ռ�������㣬ʹ���˲����ֵ����
		filter_acc_x_e=shell_out-filter_acc_x;
	core_p_out = row_runvalue.now_acc_error * pid_value.core_p;
	if(control_speed>=100)
	{
		row_runvalue.acc_error_integral += row_runvalue.now_acc_error;
		if(row_runvalue.acc_error_integral > CORE_INTEGRAL_LIMIT)
			row_runvalue.acc_error_integral = CORE_INTEGRAL_LIMIT;
		else if(row_runvalue.acc_error_integral < -CORE_INTEGRAL_LIMIT)
			row_runvalue.acc_error_integral = -CORE_INTEGRAL_LIMIT;
	}
	else
		row_runvalue.acc_error_integral = 0;
	core_i_out = row_runvalue.acc_error_integral * pid_value.core_i;
	if(!fx_use)
		row_runvalue.acc_error_differential = row_runvalue.now_acc_error - row_runvalue.old_acc_error;
	else
		row_runvalue.acc_error_differential = filter_acc_x_e - old_filter_acc_x_e;
	core_d_out = row_runvalue.acc_error_differential * pid_value.core_d;
	row_out = core_p_out + core_i_out + core_d_out;
	if(row_out>PID_OUT_LIMIT) row_out=PID_OUT_LIMIT;
	if(row_out<-PID_OUT_LIMIT) row_out=-PID_OUT_LIMIT;
	
	//yaw
	yaw_runvalue.now_angle_error = pid_value.yaw - sensor_value.ang_y;
	shell_p_out = yaw_runvalue.now_angle_error * pid_value.shell_p;
	shell_d_out = (yaw_runvalue.now_angle_error - yaw_runvalue.old_angle_error) * pid_value.shell_d;
	yaw_out = shell_p_out + shell_d_out;
	
	//update data
	if(!yfilter_flag)
		pitch_runvalue.old_acc_error = pitch_runvalue.now_acc_error;
	else
	{
		fy_use=1;
		old_filter_acc_y_e = filter_acc_y_e;
	}
	if(!xfilter_flag)
		row_runvalue.old_acc_error = row_runvalue.now_acc_error;
	else
	{
		fx_use=1;
		old_filter_acc_x_e = filter_acc_x_e;
	}
	yaw_runvalue.old_angle_error = yaw_runvalue.now_angle_error;
	
	speed1=BASE_SPEED+control_speed+(int)row_out;
	if(speed1>2000) speed1=2000;
	if(speed1<1000) speed1=1000;
	speed2=BASE_SPEED+control_speed-(int)row_out;
	if(speed2>2000) speed2=2000;
	if(speed2<1000) speed2=1000;
	speed3=BASE_SPEED+control_speed-(int)row_out;
	if(speed3>2000) speed3=2000;
	if(speed3<1000) speed3=1000;
	speed4=BASE_SPEED+control_speed+(int)row_out;
	if(speed4>2000) speed4=2000;
	if(speed4<1000) speed4=1000;
	
	set_motor_speed(speed1,speed2,speed3,speed4);
	//printf("ym: %d   speed1: %d      speed2: %d\r\n",control_speed,speed1,speed2);
}