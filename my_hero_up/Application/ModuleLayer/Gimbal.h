#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "rc_protocol.h"
#include "imu_sensor.h"
#include "rm_motor.h"
#include "KT_motor.h"
#include "motor.h"
#include "Communicate.h" 

typedef struct{
	
	KT_motor_t* y_motor;
	
	float y_imu_speed;
	float y_imu_angle;
	
	float rc_y_gyro_k;
	float key_y_gyro_k;
}Gimbal_y_t;

typedef struct{
	
	Motor_RM_t* p_mec;
	Motor_RM_t* p_gyro;
	
	float p_imu_speed;
	float p_imu_angle;

	float rc_p_mec_k;
	float rc_p_gyro_k;
	float key_p_mec_k;
	float key_p_gyro_k;
}Gimbal_p_t;

typedef struct{
	Gimbal_y_t gimbal_y_motor;
	
	Gimbal_p_t gimbal_p_motor;
	
	uint8_t gimbal_mode;
	
	uint8_t zero_bias_flag;
	
	float x_offset;
	float y_offset;
	float z_offset;
	
	float y_included_angle;
	float p_included_angle;
	
}Gimbal_t;

extern Gimbal_t gimbal_motor;

extern rc_sensor_t rc_sensor;
extern imu_sensor_t imu_sensor;


#define Y_ZERO_ANGLE  17463.f
#define P_ZERO_ANGLE  2950.f
#define P_MEC_ANGLE_MAX  3940.f
#define P_MEC_ANGLE_MIN  2770.f

#define P_GYRO_ANGLE_MAX  (gimbal_motor->gimbal_p_motor.p_imu_angle+((P_MEC_ANGLE_MAX-P_ZERO_ANGLE)-gimbal_motor->p_included_angle)*360.f/8192.f)
#define P_GYRO_ANGLE_MIN  (gimbal_motor->gimbal_p_motor.p_imu_angle-(gimbal_motor->p_included_angle-(P_MEC_ANGLE_MIN-P_ZERO_ANGLE))*360.f/8192.f)

float Imu_Data_Contrary_Menage(float imu_data);
void Gyro_zero_bias(Gimbal_t* gimbal_motor);
void Gimbal_Init(Gimbal_t* gimbal_motor);
void Gimbal_Remote_Receive(Gimbal_t* gimbal_motor);
void Gimbal_Send(Gimbal_t* gimbal_motor);
void Gimbal_PID_Calculate(Gimbal_t* gimbal_motor);
void Gimbal_Heart_Beat(Gimbal_t* gimbal_motor);
void Gimbal_Sleep(Gimbal_t* gimbal_motor);
void Gimbal_Drive(Gimbal_t* gimbal_motor);





#endif
