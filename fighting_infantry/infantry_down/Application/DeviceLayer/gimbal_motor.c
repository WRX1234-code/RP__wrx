#include "gimbal_motor.h"
#include "DM_Motor.h"
#include "motor_def.h"
/*yawµç»ú*/
Motor_DM_Born_Info_t Yaw_Motor_Born_Info =
{
	.type = _J4310_,
	.stdId = 0x002,
	
	.hcan = &hfdcan2,

};


motor_pid_t Yaw_Motor_Mec_Pid = {
	.angle = {
		.kp = 1300,
	  .ki = 0.3,
	  .kd = 0,
	  .integral_max = 200,
	  .out_max = 600,
	  .a = 1,
	},
	.speed = {
		.kp = 0.05,
	  .ki = 0,
	  .kd = 0,
	  .integral_max = 0,
	  .out_max = 50,
	  .a = 1,
	},
};

motor_pid_t Yaw_Motor_Gyro_Pid = {
	.angle = {
		.kp = 12,
	  .ki = 0.05,
	  .kd = 0,
	  .integral_max = 200,
	  .out_max = 600,
	  .a = 0.3,
	},
	.speed = {
		.kp = 0.04,
	  .ki = 0,
	  .kd = 0,
	  .integral_max = 0,
	  .out_max = 50,
	  .a = 0.3,
	},
};


Motor_DM_Rx_Info_t Yaw_Motor_Rx_Info;

Motor_DM_Tx_Info_t Yaw_Motor_Tx_Info;

Motor_DM_State_t Yaw_Motor_State;

Motor_DM_Pid_t Yaw_Motor_pid;

Motor_DM_t Yaw_Motor = 
{
	.born_info = &Yaw_Motor_Born_Info,
	
	.rx_info = &Yaw_Motor_Rx_Info,
	
	.tx_info = &Yaw_Motor_Tx_Info,
	
	.state = &Yaw_Motor_State,
	
	.pid = &Yaw_Motor_pid,
	
	.single_init = &DM_Single_Motor_Init,
};


void Yaw_Motor_Init()
{
	Yaw_Motor.single_init(&Yaw_Motor);
	motor_pid_init(&Yaw_Motor.pid->mec_pid,Yaw_Motor_Mec_Pid);
	motor_pid_init(&Yaw_Motor.pid->gyro_pid,Yaw_Motor_Gyro_Pid);
	
}

