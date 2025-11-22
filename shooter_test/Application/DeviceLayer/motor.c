
/* Includes ------------------------------------------------------------------*/
#include "motor.h"


/* Private variables ---------------------------------------------------------*/
// 添加电机时需要初始化的参数：3508还是6020，CAN1还是CAN2，以及电机的接收ID；
// CAN发送数组包含了用CAN1还是CAN2，发送数据下标在电机Motor_SendData判断。
//

drv_can_t rm_motor_driver[] = {
	[GIMB_P] = {
		.can_id = DRV_CAN2,
		.rx_id = ID_GIMB_P,  //0x204+电机id：2
	}
};
//发送ID和接收ID都可以单独设置,它的回馈报文的第一个字节是我们发送给它的ID，它的回馈报文ID可以单独设置
drv_can_t ht_motor_drive={
		.rx_id = 0x0B,
		.tx_id =0x09,
		.can_id = DRV_CAN1,
};



/*PID结构体定义------------------------------------------------*/
// 注意定义了之后需要在rm_motor_list_init用rm_motor_pid_init初始化

/*HT_start*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

Motor_HT_Born_Info_t L_Wheel_Born_Info = 
{	
	.stdId = 0x009,//电机控制报文ID
	.hcan = &hcan1,//使用的Can总线
	.order_correction = 0,//电机总角度的正方向为顺时针
};
Motor_HT_Rx_Info_t L_Wheel_Rx_Info_t;
Motor_HT_Tx_Info_t L_Wheel_Tx_Info_t;
Motor_HT_State_t L_Wheel_State_t;
Motor_HT_t L_Wheel = 
{
	.born_info = &L_Wheel_Born_Info,
	
	.rx_info = &L_Wheel_Rx_Info_t,
	
	.tx_info = &L_Wheel_Tx_Info_t,
	
	.state = &L_Wheel_State_t,
	
	.single_init = &HT_Single_Motor_Init,
};
/*HT_end*/

/*DM_start*/
Motor_DM_Born_Info_t Yaw_Born_Info =
{
	.stdId = 0x001,//电机控制报文ID
	
	.hcan = &hcan1,//使用的Can总线

};

Motor_DM_Rx_Info_t Yaw_Rx_Info_t;

Motor_DM_Tx_Info_t Yaw_Tx_Info_t;

Motor_DM_State_t Yaw_State_t;

Motor_DM_t Yaw_Motor = 
{
	.born_info = &Yaw_Born_Info,
	
	.rx_info = &Yaw_Rx_Info_t,
	
	.tx_info = &Yaw_Tx_Info_t,
	
	.state = &Yaw_State_t,
	
	.single_init = &DM_Single_Motor_Init,
};
/*DM_end*/

/*RM START*/

Motor_RM_Born_Info_t Shooter_Motor_Born[SHOOTER_MOTOR_CNT]={
	[FRIC_UP]={
		.rxId = 0,
	  .hcan = &hcan1,
	  .type = _2006_Single,
	  .stdId = 0x200,
	},
	[FRIC_R]={
		.rxId = 1,
	  .hcan = &hcan1,
	  .type = _2006_Single,
	  .stdId = 0x200,
	},
	[FRIC_L]={
		.rxId = 2,
	  .hcan = &hcan1,
	  .type = _2006_Single,
	  .stdId = 0x200,
	},
	[DIAL]={
		.rxId = 3,
	  .hcan = &hcan1,
	  .type = _2006_Single,
	  .stdId = 0x200,
	},
	[PITCH]={
		.rxId = 1,
	  .hcan = &hcan1,
	  .type = _6020_Single,
	  .stdId = 0x1FF,
	},
};

pid_ctrl_t Fric_Speed_Pid={
	.kp = 10.f,
	.ki = 0.05f,
  .kd = 1.f,
  .integral_max = 6000.f,
  .out_max = 8000.f,
};

pid_ctrl_t Dial_Speed_Pid={
	.kp = 9.5f,
	.ki = 0.f,
  .kd = 0.f,
  .integral_max = 6000.f,
  .out_max = 8000.f,
};

pid_ctrl_t Dial_Angle_Inner_Pid={
	.kp = 20.f,
	.ki = 0.2f,
  .kd = 0,
  .integral_max = 6000.f,
  .out_max = 8000.f,
};

pid_ctrl_t Dial_Angle_Outer_Pid={
	.kp = 0.3f,
	.ki = 0,
  .kd = 0,
  .integral_max = 6000.f,
  .out_max = 8000.f,
};

pid_ctrl_t Pitch_Angle_Inner_Pid={
	.kp = 0,              //200.f,
	.ki = 0,              //0.18f,
  .kd = 0,
  .integral_max = 6000.f,
  .out_max = 8000.f,
};

pid_ctrl_t Pitch_Angle_Outer_Pid={
	.kp = 0,              //1.f,
	.ki = 0,
  .kd = 0,
  .integral_max = 6000.f,
  .out_max = 8000.f,
};

pid_ctrl_t Blank_Speed_Pid[SHOOTER_MOTOR_CNT-1]={0};
pid_ctrl_t Blank_Angle_Outer_Pid[SHOOTER_MOTOR_CNT-2]={0};
pid_ctrl_t Blank_Angle_Inner_Pid[SHOOTER_MOTOR_CNT-2]={0};

Motor_RM_Tx_Info_t Shooter_Motor_Tx[SHOOTER_MOTOR_CNT];

Motor_RM_State_t Shooter_Motor_State[SHOOTER_MOTOR_CNT];

Motor_RM_Rx_Info_t Shooter_Motor_Rx[SHOOTER_MOTOR_CNT];

Motor_RM_Ctrl_Info_t Shooter_Motor_Ctrl[SHOOTER_MOTOR_CNT]={
	[FRIC_UP]={
		.speed_ctrl=&Blank_Speed_Pid[FRIC_UP],
		.angle_ctrl_outer=&Blank_Angle_Outer_Pid[FRIC_UP],
		.angle_ctrl_inner=&Blank_Angle_Inner_Pid[FRIC_UP],
	},
	[FRIC_R]={
		.speed_ctrl=&Blank_Speed_Pid[FRIC_R],
		.angle_ctrl_outer=&Blank_Angle_Outer_Pid[FRIC_R],
		.angle_ctrl_inner=&Blank_Angle_Inner_Pid[FRIC_R],
	},
	[FRIC_L]={
		.speed_ctrl=&Blank_Speed_Pid[FRIC_L],
		.angle_ctrl_outer=&Blank_Angle_Outer_Pid[FRIC_L],
		.angle_ctrl_inner=&Blank_Angle_Inner_Pid[FRIC_L],
	},
	[DIAL]={
		.speed_ctrl=&Dial_Speed_Pid,
		.angle_ctrl_outer=&Dial_Angle_Outer_Pid,
		.angle_ctrl_inner=&Dial_Angle_Inner_Pid,
	},
	[PITCH]={
		.speed_ctrl=&Blank_Speed_Pid[SHOOTER_MOTOR_CNT-2],
		.angle_ctrl_outer=&Pitch_Angle_Outer_Pid,
		.angle_ctrl_inner=&Pitch_Angle_Inner_Pid,
	},
};

Motor_RM_t Fric_Up= 
{
	.born_info = &Shooter_Motor_Born[FRIC_UP],
	
	.rx_info = &Shooter_Motor_Rx[FRIC_UP],
	
	.tx_info = &Shooter_Motor_Tx[FRIC_UP],

  .state = &Shooter_Motor_State[FRIC_UP],
	
	.single_init = RM_Motor_Init,
	
	.ctrl =&Shooter_Motor_Ctrl[FRIC_UP],
};

Motor_RM_t Fric_R= 
{
	.born_info = &Shooter_Motor_Born[FRIC_R],
	
	.rx_info = &Shooter_Motor_Rx[FRIC_R],
	
	.tx_info = &Shooter_Motor_Tx[FRIC_R],

  .state = &Shooter_Motor_State[FRIC_R],
	
	.single_init = RM_Motor_Init,
	
	.ctrl = &Shooter_Motor_Ctrl[FRIC_R],
};

Motor_RM_t Fric_L= 
{
	.born_info = &Shooter_Motor_Born[FRIC_L],
	
	.rx_info = &Shooter_Motor_Rx[FRIC_L],
	
	.tx_info = &Shooter_Motor_Tx[FRIC_L],

  .state = &Shooter_Motor_State[FRIC_L],
	
	.single_init = RM_Motor_Init,
	
	.ctrl = &Shooter_Motor_Ctrl[FRIC_L],
};

Motor_RM_t Dial= 
{
	.born_info = &Shooter_Motor_Born[DIAL],
	
	.rx_info = &Shooter_Motor_Rx[DIAL],
	
	.tx_info = &Shooter_Motor_Tx[DIAL],

  .state = &Shooter_Motor_State[DIAL],
	
	.single_init = RM_Motor_Init,
	
	.ctrl = &Shooter_Motor_Ctrl[DIAL],
};

Motor_RM_t Pitch= 
{
	.born_info = &Shooter_Motor_Born[PITCH],
	
	.rx_info = &Shooter_Motor_Rx[PITCH],
	
	.tx_info = &Shooter_Motor_Tx[PITCH],

  .state = &Shooter_Motor_State[PITCH],
	
	.single_init = RM_Motor_Init,
	
	.ctrl = &Shooter_Motor_Ctrl[PITCH],
};

Motor_RM_Group_t RM_Group =
{
	.motor[0] = &Fric_Up,
	
	.motor[1] = &Fric_R,
	
	.motor[2] = &Fric_L,
	
	.motor[3] = &Dial,
	
	.stdId=0x200,
	
	.hcan=&hcan1,
	
	.group_init = RM_Group_Motor_Init,
};

/*RM END*/
KT_motor_t kt_motor[] = {
	[0] = {
		.KT_motor_info = {
			.tx_info = {
				.angle_single_Control = 0,
				.angle_single_Control_maxSpeed = 0,
				.angle_single_Control_spinDirection = 0,
				.angle_add_Control = 0,
				.angle_add_Control_maxSpeed = 0,
				.angle_sum_Control = 0,
				.angle_sum_Control_maxSpeed = 0,
				.iqControl = 0,
				.speedControl = 0,
			},
			.id = {
				.tx_id = ID_GIMB_YAW,
				.rx_id = 0x88,
				.drive_type = M_CAN1,
				.motor_type = KT9015,
			},
		},
		.init = KT_motor_class_init,
	},
};

/* Exported functions --------------------------------------------------------*/
void rm_motor_list_init()
{
	/*电机信息初始化*/
	RM_Group.group_init(&RM_Group);
	Pitch.single_init(&Pitch);
	
	for(uint8_t i=0;i<SHOOTER_MOTOR_CNT-2;i++)
	{
		shooter_pid_init(RM_Group.motor[i]->ctrl->speed_ctrl,Fric_Speed_Pid); 
		
	}
}

void kt_motor_list_init()
{
	kt_motor[0].init(&kt_motor[0]);
	
	
}
void dm_motor_list_init()
{
	Yaw_Motor.single_init(&Yaw_Motor);
	
}

void ht_motor_list_init()
{
	L_Wheel.single_init(&L_Wheel);
	
}

void rm_motor_list_heart_beat()
{
	RM_Group.group_heartbeat(&RM_Group);
}


