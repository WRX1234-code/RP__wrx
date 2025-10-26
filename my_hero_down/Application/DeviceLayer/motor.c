
/* Includes ------------------------------------------------------------------*/
#include "motor.h"



/* Private variables ---------------------------------------------------------*/
// 添加电机时需要初始化的参数：3508还是6020，CAN1还是CAN2，以及电机的接收ID；
// CAN发送数组包含了用CAN1还是CAN2，发送数据下标在电机Motor_SendData判断。
//

drv_can_t rm_motor_driver[] = {
//	[GIMB_P] = {
//		.can_id = DRV_CAN2,
//		.rx_id = ID_GIMB_P,  //0x204+电机id：2
//	}
	  [chas_lf]={
			.can_id=DRV_CAN2,
			.rx_id=ID_Chas_lf,
		},
		[chas_rf]={
			.can_id=DRV_CAN2,
			.rx_id=ID_Chas_rf,
		},
		[chas_lb]={
			.can_id=DRV_CAN2,
			.rx_id=ID_Chas_lb,
		},
		[chas_rb]={
			.can_id=DRV_CAN2,
			.rx_id=ID_Chas_rb,
		},
};
//发送ID和接收ID都可以单独设置,它的回馈报文的第一个字节是我们发送给它的ID，它的回馈报文ID可以单独设置
drv_can_t ht_motor_drive={
		.rx_id = 0x0B,
		.tx_id =0x09,
		.can_id = DRV_CAN1,
};



/*PID结构体定义------------------------------------------------*/
// 注意定义了之后需要在rm_motor_list_init用rm_motor_pid_init初始化

motor_pid_t GIMB_P_mec = {
	.speed.kp = 0,
	.speed.ki = 0,
	.speed.kd = 0,
	.speed.integral_max = 3000,
	.speed.out_max = 28000,
	.angle.kp = 0, // 0.45
	.angle.ki = 0,
	.angle.kd = 0,
	.angle.integral_max = 0,
	.angle.out_max = 500,
};
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
Motor_RM_Born_Info_t chas_born_info[RM_MOTOR_LIST]={
	[chas_lf]={
		.hcan=&hcan2,
	  .order_correction=1,
	  .stdId=0x200,
	  .type=_3508_Single,
		.rxId=0,
	},
	[chas_rf]={
		.hcan=&hcan2,
	  .order_correction=1,
	  .stdId=0x200,
	  .type=_3508_Single,
		.rxId=1,
	},
	[chas_lb]={
		.hcan=&hcan2,
	  .order_correction=1,
	  .stdId=0x200,
	  .type=_3508_Single,
		.rxId=2,
	},
	[chas_rb]={
		.hcan=&hcan2,
	  .order_correction=1,
	  .stdId=0x200,
	  .type=_3508_Single,
		.rxId=3,
	}
	
};

pid_ctrl_t chas_speed_pid={
	.kp = 8,
	.ki = 0.33,
	.kd = 0,
	.integral_max = 6000,
	.out_max = 15000,
};

pid_ctrl_t chas_angle_inner_pid={
	.kp = 8,
	.ki = 0.1,
	.kd = 0,
	.integral_max = 6000,
	.out_max = 10000,
};

pid_ctrl_t chas_angle_outer_pid={
	.kp = 1,
	.ki = 0,
	.kd = 0,
	.integral_max = 0,
	.out_max = 10000,
};

pid_ctrl_t chassis_speed_pid[RM_MOTOR_LIST];
pid_ctrl_t chassis_angle_inner_pid[RM_MOTOR_LIST];
pid_ctrl_t chassis_angle_outer_pid[RM_MOTOR_LIST];

Motor_RM_Rx_Info_t chas_rx_info[RM_MOTOR_LIST];
Motor_RM_Tx_Info_t chas_tx_info[RM_MOTOR_LIST];
Motor_RM_Ctrl_Info_t chas_ctrl[RM_MOTOR_LIST]={
	[chas_lf]={
		.speed_ctrl=&chassis_speed_pid[chas_lf],
		.angle_ctrl_inner=&chassis_angle_inner_pid[chas_lf],
		.angle_ctrl_outer=&chassis_angle_outer_pid[chas_lf],
	},
	[chas_rf]={
		.speed_ctrl=&chassis_speed_pid[chas_rf],
		.angle_ctrl_inner=&chassis_angle_inner_pid[chas_rf],
		.angle_ctrl_outer=&chassis_angle_outer_pid[chas_rf],
	},	
	[chas_lb]={
		.speed_ctrl=&chassis_speed_pid[chas_lb],
		.angle_ctrl_inner=&chassis_angle_inner_pid[chas_lb],
		.angle_ctrl_outer=&chassis_angle_outer_pid[chas_lb],
	},	
	[chas_rb]={
		.speed_ctrl=&chassis_speed_pid[chas_rb],
		.angle_ctrl_inner=&chassis_angle_inner_pid[chas_rb],
		.angle_ctrl_outer=&chassis_angle_outer_pid[chas_rb],
	},	
};

Motor_RM_State_t chas_state[RM_MOTOR_LIST];



Motor_RM_t rm_motor[]={
	[chas_lf]={
		.born_info=&chas_born_info[chas_lf],
		.rx_info=&chas_rx_info[chas_lf],
		.tx_info=&chas_tx_info[chas_lf],
		.state=&chas_state[chas_lf],
		.single_init=&RM_Motor_Init,
		.ctrl=&chas_ctrl[chas_lf],
	},
	[chas_rf]={
		.born_info=&chas_born_info[chas_rf],
		.rx_info=&chas_rx_info[chas_rf],
		.tx_info=&chas_tx_info[chas_rf],
		.state=&chas_state[chas_rf],
		.single_init=&RM_Motor_Init,
		.ctrl=&chas_ctrl[chas_rf],
	},
	[chas_lb]={
		.born_info=&chas_born_info[chas_lb],
		.rx_info=&chas_rx_info[chas_lb],
		.tx_info=&chas_tx_info[chas_lb],
		.state=&chas_state[chas_lb],
		.single_init=&RM_Motor_Init,
		.ctrl=&chas_ctrl[chas_lb],
	},
	[chas_rb]={
		.born_info=&chas_born_info[chas_rb],
		.rx_info=&chas_rx_info[chas_rb],
		.tx_info=&chas_tx_info[chas_rb],
		.state=&chas_state[chas_rb],
		.single_init=&RM_Motor_Init,
		.ctrl=&chas_ctrl[chas_rb],
	}
};




Motor_RM_Born_Info_t R_Fric_Born = 
{
	.rxId = 0,
	
	.hcan = &hcan1,
	
	.type = _6020_Single,
	
	.stdId = 0x1FE,
};

Motor_RM_Tx_Info_t R_Fric_Tx;

Motor_RM_State_t R_Fric_State;

Motor_RM_Rx_Info_t R_Fric_Rx;

pid_ctrl_t R_Fric_Speed_Ctrl = 
{
	.kp = 10.f,//
	.ki = 0.2f,
	.kd = 0.f,
	.integral_max = 6000.f,
	.out_max = 8000.f,//
};

Motor_RM_Ctrl_Info_t R_Fric_Ctrl = 
{
	.speed_ctrl = &R_Fric_Speed_Ctrl,
};

Motor_RM_t R_Fric = 
{
	.born_info = &R_Fric_Born,
	
	.rx_info = &R_Fric_Rx,
	
	.tx_info = &R_Fric_Tx,

  .state = &R_Fric_State,
	
	.single_init = RM_Motor_Init,
	
	.ctrl = &R_Fric_Ctrl,
};

Motor_RM_Group_t RM_Group =
{
	.motor[0] = &R_Fric,
	
	.motor[1] = NULL,
	
	.motor[2] = NULL,
	
	.motor[3] = NULL,
	
	.stdId=0x1FE,
	
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
//	R_Fric.single_init(&R_Fric);
//	RM_Group.group_init(&RM_Group);
	
	 
	for(uint8_t i=0;i<RM_MOTOR_LIST;i++)
	{
//		RM_Motor_Init(&rm_motor[i]);
//		rm_motor->single_init(&rm_motor[i]);
		rm_motor[i].single_init(&rm_motor[i]);
			
//		rm_motor[i].born_info->rxId=0x201+i;
		
		my_motor_pid_init(&chassis_speed_pid[i],chas_speed_pid); 
	  my_motor_pid_init(&chassis_angle_inner_pid[i],chas_angle_inner_pid); 
	  my_motor_pid_init(&chassis_angle_outer_pid[i],chas_angle_outer_pid); 
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


