#include "Launch_motor.h"
#include "rm_motor.h"
#include "motor_def.h"

/*-------------------²¦ÅÌµç»ú-------------------*/

Motor_RM_Born_Info_t Dial_Motor_Born = 
{
	.rxId = 3,
	
	.hcan = &hfdcan1,
	
	.type = _2006_Single,
	
	.stdId = 0x200,
	
	.order_correction = -1,

};

pid_ctrl_t Dial_Motor_Speed_Ctrl = {
	.kp = 15.f,
	.ki = 0.1f,
	.kd = 0.f,
	.integral_max = 80000,
	.out_max = 10000,
	.a = 0.2f,
};

pid_ctrl_t Dail_Motor_Angle_Outer_Ctrl = {
	.kp = 0.1f,//0.3f
	.ki = 0.2f,//0.05f
	.kd = 0,
	.integral_max = 1000,
	.out_max = 8000,
	.a = 1,
};

pid_ctrl_t Dail_Motor_Angle_Inner_Ctrl = {
	.kp = 7,
	.ki = 0,
	.kd = 0,
	.integral_max = 0,
	.out_max = 10000,
	.a = 0.3f,
};

Motor_RM_Tx_Info_t Dial_Motor_Tx_Info;

Motor_RM_State_t Dial_Motor_State;

Motor_RM_Rx_Info_t Dial_Motor_Rx_Info;

Motor_RM_Ctrl_Info_t Dial_Motor_Ctrl_Info = {
	.speed_ctrl = &Dial_Motor_Speed_Ctrl,
	.angle_ctrl_inner = &Dail_Motor_Angle_Inner_Ctrl,
	.angle_ctrl_outer = &Dail_Motor_Angle_Outer_Ctrl,
	
};

Motor_RM_t Dial_Motor = 
{
	.born_info = &Dial_Motor_Born,
	
	.rx_info = &Dial_Motor_Rx_Info,
	
	.tx_info = &Dial_Motor_Tx_Info,

  .state = &Dial_Motor_State,
	
	.ctrl = &Dial_Motor_Ctrl_Info,
	
	.single_init = RM_Motor_Init,
	
};

void Dial_Motor_Init()
{
	Dial_Motor.single_init(&Dial_Motor);
	
}
