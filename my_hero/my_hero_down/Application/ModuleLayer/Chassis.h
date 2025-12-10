#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "rm_motor.h"
#include "rc_sensor.h"
#include "communicate.h"

typedef struct{
	Motor_RM_t* wheel_lf;
	Motor_RM_t* wheel_rf;
	Motor_RM_t* wheel_lb;
	Motor_RM_t* wheel_rb;
	float chassis_speed_x;
	float	chassis_speed_y;
	float chassis_speed_w;
	pid_ctrl_t w;
	uint8_t chassis_mode;
	
	float chassis_output[4];
}Chassis_t;

extern Communicate_Chassis_Target_t communicate_chassis_target;

extern Chassis_t chassis_wheel;

extern rc_sensor_info_t rc_sensor_info;
extern rc_sensor_t rc_sensor;

void Chassis_Wheel_Init(Chassis_t* chassis_wheel);
void Chassis_Remote_Receive(Chassis_t* chassis_wheel);
void Chassis_Speed_Calculate(Chassis_t* chassis_wheel);
void Chassis_Send(Chassis_t* chassis_wheel);
void Chassis_PID_Speed_Calculate(Chassis_t* chassis_wheel);

float Calculate_Predicted_Power(float i, float w);
float Calculate_Current_Out(float target_power,float w,int16_t raw_curret);
void New_Chassis_Power_Limit(Chassis_t chassis_wheel);

#endif

