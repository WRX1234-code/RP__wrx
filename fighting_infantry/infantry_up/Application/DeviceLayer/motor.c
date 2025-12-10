
/* Includes ------------------------------------------------------------------*/
#include "motor.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;


Motor_DM_Born_Info_t Pitch_Born_Info =
{
	.stdId = 0x001,//电机控制报文ID
	
	.hcan = &hcan1,//使用的Can总线

};


motor_pid_t Pitch_Mec_Pid = {
	.speed={
		.kp = 0,
  	.ki = 0,
  	.kd = 0,
	  .integral_max = 0,
    .out_max = 0,
    .filter_value = 1,
	},
	.angle={
		.kp = 0,
  	.ki = 0,
  	.kd = 0,
	  .integral_max = 0,
    .out_max = 0,
    .filter_value = 1,
	},
};


motor_pid_t Pitch_Gyro_Pid = {
	.speed={
		.kp = 0,
  	.ki = 0,
  	.kd = 0,
	  .integral_max = 0,
    .out_max = 0,
    .filter_value = 1,
	},
	.angle={
		.kp = 0,
  	.ki = 0,
  	.kd = 0,
	  .integral_max = 0,
    .out_max = 0,
    .filter_value = 1,
	},
}; 


Motor_DM_Rx_Info_t Pitch_Rx_Info_t;

Motor_DM_Tx_Info_t Pitch_Tx_Info_t;

Motor_DM_State_t Pitch_State_t;

Motor_DM_Pid_t Pitch_Pid_t;
	
Motor_DM_t Pitch_Motor = 
{
	.born_info = &Pitch_Born_Info,
	
	.rx_info = &Pitch_Rx_Info_t,
	
	.tx_info = &Pitch_Tx_Info_t,
	
	.state = &Pitch_State_t,
	
	.pid = &Pitch_Pid_t,
	
	.single_init = &DM_Single_Motor_Init,
};


//
pid_ctrl_t Fric_Speed_Pid[FRIC_MOTOR_LIST] ={
	[FRIC_UP] = {
		.kp = 0,
    .ki = 0,
    .kd = 0,
    .integral_max = 0,
    .out_max = 0,
    .filter_value = 1,
	},
	[FRIC_R] = {
		.kp = 0,
    .ki = 0,
    .kd = 0,
    .integral_max = 0,
    .out_max = 0,
    .filter_value = 1,
	},
	[FRIC_L] = {
		.kp = 0,
    .ki = 0,
    .kd = 0,
    .integral_max = 0,
    .out_max = 0,
    .filter_value = 1,
	},

};

Motor_RM_Born_Info_t Fric_Up_Born={
	.order_correction=1,
  .rxId = 0,
	.hcan = &hcan2,
	.type = _3508_Single,
	.stdId = 0x200,
};

Motor_RM_Born_Info_t Fric_R_Born={
	.order_correction=1,
  .rxId = 1,
	.hcan = &hcan2,
	.type = _3508_Single,
	.stdId = 0x200,
};

Motor_RM_Born_Info_t Fric_L_Born={
	.order_correction=1,
  .rxId = 2,
	.hcan = &hcan2,
	.type = _3508_Single,
	.stdId = 0x200,
};

Motor_RM_Tx_Info_t Fric_Tx[FRIC_MOTOR_LIST];

Motor_RM_State_t Fric_State[FRIC_MOTOR_LIST];

Motor_RM_Rx_Info_t Fric_Rx[FRIC_MOTOR_LIST];

Motor_RM_Ctrl_Info_t RM_Ctrl[FRIC_MOTOR_LIST];

Motor_RM_t Fric_Up_Motor = 
{
	.born_info = &Fric_Up_Born,
	
	.rx_info = &Fric_Rx[FRIC_UP],
	
	.tx_info = &Fric_Tx[FRIC_UP],

  .state = &Fric_State[FRIC_UP],
	
	.single_init = RM_Motor_Init,
	
	.ctrl = &RM_Ctrl[FRIC_UP],
};

Motor_RM_t Fric_R_Motor = 
{
	.born_info = &Fric_R_Born,
	
	.rx_info = &Fric_Rx[FRIC_R],
	
	.tx_info = &Fric_Tx[FRIC_R],

  .state = &Fric_State[FRIC_R],
	
	.single_init =RM_Motor_Init,
	
	.ctrl = &RM_Ctrl[FRIC_R],
};

Motor_RM_t Fric_L_Motor = 
{
	.born_info = &Fric_L_Born,
	
	.rx_info = &Fric_Rx[FRIC_L],
	
	.tx_info = &Fric_Tx[FRIC_L],

  .state = &Fric_State[FRIC_L],
	
	.single_init = RM_Motor_Init,
	
	.ctrl = &RM_Ctrl[FRIC_L],
};


Motor_RM_Group_t RM_Group={
	.motor={
		[0]= &Fric_Up_Motor,
		[1]= &Fric_R_Motor,
		[2]= &Fric_L_Motor,
		[3]= NULL,
	},
	.stdId=0x200,
	.hcan=&hcan1,
	
	.group_init=RM_Group_Motor_Init,
};


/* Exported functions --------------------------------------------------------*/
void rm_motor_list_init()
{
	/*电机信息初始化*/

	RM_Group.group_init(&RM_Group);
	for(uint8_t i = 0;i < FRIC_MOTOR_LIST;i++)
	{
		rm_motor_pid_init(RM_Group.motor[i]->ctrl->speed_ctrl,Fric_Speed_Pid[i]);
	}
	
	
}

void kt_motor_list_init()
{
	
}
void dm_motor_list_init()
{
	Pitch_Motor.single_init(&Pitch_Motor);
	motor_pid_init(&Pitch_Motor.pid->mec_pid,Pitch_Mec_Pid);
	motor_pid_init(&Pitch_Motor.pid->gyro_pid,Pitch_Gyro_Pid);
	
}

void ht_motor_list_init()
{
	
	
}

void rm_motor_list_heart_beat()
{
	RM_Group.group_heartbeat(&RM_Group);

}
