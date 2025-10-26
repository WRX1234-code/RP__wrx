#ifndef __COMMUNICATE_H
#define __COMMUNICATE_H

#include "rc_sensor.h"

typedef struct{
	float mec_angle_err;
	float gyro_angle_err;
	
}Public_Message;

typedef struct{
	int16_t x_target_speed;
	int16_t y_target_speed;
	int16_t w_target_speed;
	uint8_t heart_state;
	
}Communicate_Chassis_Target_t;

extern rc_sensor_info_t rc_sensor_info;
extern rc_sensor_t rc_sensor;

extern uint8_t buff[8];
extern Communicate_Chassis_Target_t communicate_chassis_target;

extern float gyro_cycle_speed;

extern uint8_t heart_cnt;
extern uint8_t gimbal_heart_state;

extern Public_Message chassis_gimbal_share;

void Public_Message_Init(Public_Message* chassis_gimbal_share);

void Communicate_Chassis_Message(Communicate_Chassis_Target_t* chassis_message);
void Gimbal_motor_Send(void);

#endif
