#include "Communicate.h"
#include "rc_protocol.h"
#include "drv_can.h"
#include "rc_sensor.h"
#include "Gimbal.h"
#include "Shooter.h"

Public_Message chassis_gimbal_share;

uint8_t heart_cnt;
uint8_t gimbal_heart_state;
float gyro_cycle_speed;

void Public_Message_Init(Public_Message* chassis_gimbal_share)
{
	chassis_gimbal_share->mec_angle_err=0;
	chassis_gimbal_share->gyro_angle_err=0;
}

float motor_angle_err;
float angle_err;
float angle_err_raw;

Communicate_Chassis_Target_t communicate_chassis_target;

uint8_t buff[8];

void Communicate_Chassis_Message(Communicate_Chassis_Target_t* communicate_chassis_target)
{
	float k=2.0f;
	communicate_chassis_target->x_target_speed=rc_sensor.info->ch3*k;
	communicate_chassis_target->y_target_speed=rc_sensor.info->ch2*k;
	communicate_chassis_target->w_target_speed=rc_sensor.info->ch0*k;
	
	if(gimbal_motor.gimbal_mode==1)
	{
		communicate_chassis_target->x_target_speed=rc_sensor.info->ch3*k;
	  communicate_chassis_target->y_target_speed=rc_sensor.info->ch2*k;
	  communicate_chassis_target->w_target_speed=rc_sensor.info->ch0*k;
	}
	else if(gimbal_motor.gimbal_mode==2)
	{
		motor_angle_err=Y_ZERO_ANGLE-(float)gimbal_motor.gimbal_y_motor.y_motor->KT_motor_info.rx_info.encoder;
		motor_angle_err=motor_half_cycle(motor_angle_err,32768.f);
		
		angle_err=motor_angle_err/32768*4096;
		
		angle_err_raw=angle_err*3.1415926f/4096.0f;
		communicate_chassis_target->x_target_speed=communicate_chassis_target->x_target_speed*cos(angle_err_raw)-communicate_chassis_target->y_target_speed*sin(angle_err_raw);
		communicate_chassis_target->y_target_speed=communicate_chassis_target->y_target_speed*cos(angle_err_raw)+communicate_chassis_target->x_target_speed*sin(angle_err_raw);
		communicate_chassis_target->w_target_speed=angle_err*angle_err*sgn(angle_err)/4096.f*30;
	}
	else if(gimbal_motor.gimbal_mode==3)
	{
		gyro_cycle_speed=2000.f;
		communicate_chassis_target->x_target_speed=communicate_chassis_target->x_target_speed*cos(angle_err_raw)-communicate_chassis_target->y_target_speed*sin(angle_err_raw);
		communicate_chassis_target->y_target_speed=communicate_chassis_target->y_target_speed*cos(angle_err_raw)+communicate_chassis_target->x_target_speed*sin(angle_err_raw);
		communicate_chassis_target->w_target_speed=gyro_cycle_speed;
	}
	
	memcpy(buff,communicate_chassis_target,sizeof(Communicate_Chassis_Target_t));
  CAN1_SendData(0x250,buff);
	heart_cnt++;
}

void Gimbal_motor_Send(void)
{
	RM_Group1.group_set_torque(&RM_Group1);
	RM_Group2.group_set_torque(&RM_Group2);
//	shoot.dial.dial_config->single_set_torque(shoot.dial.dial_config);
	
//	gimbal_motor.gimbal_y_motor.y_motor->tx_W_cmd(gimbal_motor.gimbal_y_motor.y_motor,TORQUE_CLOSE_LOOP_ID);
	
}
