
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

pid_ctrl_t gim_p_speed={
	.kp = 70,
	.ki = 5,
	.kd = 0,
	.integral_max = 3000,
	.out_max = 28000,
};

pid_ctrl_t gim_p_mec_angle_inner={
	.kp = 600.f,    //800
	.ki = 0,
	.kd = 0,
	.integral_max = 3000,
	.out_max = 28000,
};

pid_ctrl_t gim_p_mec_angle_outer={
	.kp = 2, // 0.45
	.ki = 0,
	.kd = 0,
	.integral_max = 0,
	.out_max = 500,
};

pid_ctrl_t gim_p_gyro_angle_inner={
	.kp = 800, // 300 3 0 7 0 0   800
  .ki = 8,
  .kd = 5,
  .integral_max = 1000,
  .out_max = 28000,
};

pid_ctrl_t gim_p_gyro_angle_outer={
	.kp = 1.2,
  .ki = 0,
  .kd = 0,
  .integral_max = 0,
  .out_max = 500,
};

pid_ctrl_t gim_p_speed_pid[MOTOR_MODE_LIST];

Motor_RM_Born_Info_t gim_p_info={
	.order_correction=1,
	.rxId=1,
	.stdId=0x1FF,
  .type=_6020_Single,
	.hcan=&hcan2,
};

Motor_RM_Rx_Info_t gim_rx_info;

Motor_RM_Ctrl_Info_t gim_ctrl_info[MOTOR_MODE_LIST]={
	[MOTOR_MEC]={
		.speed_ctrl=&gim_p_speed_pid[MOTOR_MEC],
		.angle_ctrl_inner=&gim_p_mec_angle_inner,
		.angle_ctrl_outer=&gim_p_mec_angle_outer,
	},
	[MOTOR_GYRO]={
		.speed_ctrl=&gim_p_speed_pid[MOTOR_GYRO],
		.angle_ctrl_inner=&gim_p_gyro_angle_inner,
		.angle_ctrl_outer=&gim_p_gyro_angle_outer,
	},
};

Motor_RM_Tx_Info_t gim_tx_info;
Motor_RM_State_t gim_state;

Motor_RM_t gim_p_motor[]={
	[MOTOR_MEC]={
		.born_info=&gim_p_info,
	  .rx_info=&gim_rx_info,
	  .tx_info=&gim_tx_info,
	  .state=&gim_state,
	  .ctrl=&gim_ctrl_info[MOTOR_MEC],
		.single_init=RM_Motor_Init,
	},
	[MOTOR_GYRO]={
		.born_info=&gim_p_info,
	  .rx_info=&gim_rx_info,
	  .tx_info=&gim_tx_info,
	  .state=&gim_state,
	  .ctrl=&gim_ctrl_info[MOTOR_GYRO],
		.single_init=RM_Motor_Init,
	},	
		
};
	

pid_ctrl_t first_fric_speed[3]={
	[0]={
		.kp = 27,
	  .ki = 0.5,
	  .kd = 0,
	  .integral_max = 6000,
	  .out_max = 10000,
	},
	[1]={
		.kp = 27,
	  .ki = 0.5,
	  .kd = 0,
	  .integral_max = 6000,
	  .out_max = 10000,
	},
	[2]={
		.kp = 27,
	  .ki = 0.5,
	  .kd = 0,
	  .integral_max = 6000,
	  .out_max = 10000,
	},
};
	
pid_ctrl_t second_fric_speed[3]={
  [0]={
		.kp = 27,
	  .ki = 0.5,
	  .kd = 0,
	  .integral_max = 6000,
	  .out_max = 10000,
	},
  [1]={
		.kp = 27,
	  .ki = 0.5,
	  .kd = 0,
	  .integral_max = 6000,
	  .out_max = 10000,
	},
	[2]={
		.kp = 27,
	  .ki = 0.5,
	  .kd = 0,
	  .integral_max = 6000,
	  .out_max = 10000,
	},
};

pid_ctrl_t dail_speed={
  .kp = 15,
	.ki = 0,
	.kd = 0,
	.integral_max = 8000,
	.out_max = 28000,
};

pid_ctrl_t dail_angle_inner={
  .kp = 14.74f,
	.ki = 0,
	.kd = 0,
	.integral_max = 0,
	.out_max = 20000,

};

pid_ctrl_t dail_angle_outer={
  .kp = 0.15,
	.ki = 0,
	.kd = 0,
	.integral_max = 0,
	.out_max = 8000
};

pid_ctrl_t NULL_inner[6]={0};
pid_ctrl_t NULL_outer[6]={0};


Motor_RM_Born_Info_t fric_b_up={
	.order_correction=1,
  .rxId = 0,
	.hcan = &hcan2,
	.type = _3508_Single,
	.stdId = 0x200,
};

Motor_RM_Born_Info_t fric_f_up={
	.order_correction=1,
  .rxId = 1,
	.hcan = &hcan2,
	.type = _3508_Single,
	.stdId = 0x200,
};

Motor_RM_Born_Info_t fric_b_r={
	.order_correction=1,
  .rxId = 2,
	.hcan = &hcan2,
	.type = _3508_Single,
	.stdId = 0x200,
};

Motor_RM_Born_Info_t fric_b_l={
	.order_correction=1,
  .rxId = 3,
	.hcan = &hcan2,
	.type = _3508_Single,
	.stdId = 0x200,
};

Motor_RM_Born_Info_t fric_f_r={
	.order_correction=1,
  .rxId = 0,
	.hcan = &hcan2,
	.type = _3508_Single,
	.stdId = 0x1FF,
};

Motor_RM_Born_Info_t fric_f_l={
	.order_correction=1,
  .rxId = 2,
	.hcan = &hcan2,
	.type = _3508_Single,
	.stdId = 0x1FF,
};

Motor_RM_Born_Info_t dail={
	.order_correction=1,
  .rxId = 2,
	.hcan = &hcan1,
	.type = _3508_Single,
	.stdId = 0x1FF,
};

Motor_RM_Tx_Info_t Fric_Tx[7]={0};

Motor_RM_State_t Fric_State[7]={0};

Motor_RM_Rx_Info_t Fric_Rx[7]={0};

Motor_RM_Ctrl_Info_t RM_Ctrl[7]={
	[0]={
		.speed_ctrl=&first_fric_speed[0],
		.angle_ctrl_inner=&NULL_inner[0],
		.angle_ctrl_outer=&NULL_outer[0],
	},
	[1]={
		.speed_ctrl=&second_fric_speed[0],
		.angle_ctrl_inner=&NULL_inner[1],
		.angle_ctrl_outer=&NULL_outer[1],
	},
	[2]={
		.speed_ctrl=&first_fric_speed[1],
		.angle_ctrl_inner=&NULL_inner[2],
		.angle_ctrl_outer=&NULL_outer[2],
	},
	[3]={
		.speed_ctrl=&first_fric_speed[2],
		.angle_ctrl_inner=&NULL_inner[3],
		.angle_ctrl_outer=&NULL_outer[3],
	},
	[4]={
		.speed_ctrl=&second_fric_speed[1],
		.angle_ctrl_inner=&NULL_inner[4],
		.angle_ctrl_outer=&NULL_outer[4],
	},
	[5]={
		.speed_ctrl=&second_fric_speed[2],
		.angle_ctrl_inner=&NULL_inner[5],
		.angle_ctrl_outer=&NULL_outer[5],
	},
	[6]={
		.speed_ctrl=&dail_speed,
		.angle_ctrl_inner=&dail_angle_inner,
		.angle_ctrl_outer=&dail_angle_outer,
	}

};

Motor_RM_t W_Fric_B_UP = 
{
	.born_info = &fric_b_up,
	
	.rx_info = &Fric_Rx[0],
	
	.tx_info = &Fric_Tx[0],

  .state = &Fric_State[0],
	
	.single_init = RM_Motor_Init,
	
	.ctrl = &RM_Ctrl[0],
};

Motor_RM_t W_Fric_F_UP = 
{
	.born_info = &fric_f_up,
	
	.rx_info = &Fric_Rx[1],
	
	.tx_info = &Fric_Tx[1],

  .state = &Fric_State[1],
	
	.single_init =RM_Motor_Init,
	
	.ctrl = &RM_Ctrl[1],
};

Motor_RM_t W_Fric_B_R = 
{
	.born_info = &fric_b_r,
	
	.rx_info = &Fric_Rx[2],
	
	.tx_info = &Fric_Tx[2],

  .state = &Fric_State[2],
	
	.single_init = RM_Motor_Init,
	
	.ctrl = &RM_Ctrl[2],
};

Motor_RM_t W_Fric_B_L = 
{
	.born_info = &fric_b_l,
	
	.rx_info = &Fric_Rx[3],
	
	.tx_info = &Fric_Tx[3],

  .state = &Fric_State[3],
	
	.single_init = RM_Motor_Init,
	
	.ctrl = &RM_Ctrl[3],
};

Motor_RM_t W_Fric_F_R = 
{
	.born_info = &fric_f_r,
	
	.rx_info = &Fric_Rx[4],
	
	.tx_info = &Fric_Tx[4],

  .state = &Fric_State[4],
	
	.single_init = RM_Motor_Init,
	
	.ctrl = &RM_Ctrl[4],
};

Motor_RM_t W_Fric_F_L = 
{
	.born_info = &fric_f_l,
	
	.rx_info = &Fric_Rx[5],
	
	.tx_info = &Fric_Tx[5],

  .state = &Fric_State[5],
	
	.single_init = RM_Motor_Init,
	
	.ctrl = &RM_Ctrl[5],
};

Motor_RM_t W_Dail = 
{
	.born_info = &dail,
	
	.rx_info = &Fric_Rx[6],
	
	.tx_info = &Fric_Tx[6],

  .state = &Fric_State[6],
	
	.single_init =RM_Motor_Init,
	
	.ctrl = &RM_Ctrl[6],
};

static const Motor_RM_t NULL_motor={0};

Motor_RM_Group_t RM_Group1={
	.motor={
		[0]=&W_Fric_B_UP,
		[1]=&W_Fric_F_UP,
		[2]=&W_Fric_B_R,
		[3]=&W_Fric_B_L
	},
	.stdId=0x200,
	.hcan=&hcan2,
	
	.group_init=RM_Group_Motor_Init,
};

Motor_RM_Group_t RM_Group2={
	.motor={
		[0]=&W_Fric_F_R,
	  [1]=&gim_p_motor[MOTOR_MEC],
	  [2]=&W_Fric_F_L,
		[3]=(Motor_RM_t*)&NULL_motor,
	},
	.stdId=0x1FF,
	.hcan=&hcan2,
	
	.group_init=RM_Group_Motor_Init,
};

/*RM END*/

motor_pid_t GIMB_Y_mec = {
	.speed.kp = 15.f,         //20
	.speed.ki = 0.15,         //0.5
	.speed.kd = 0,           //0
	.speed.integral_max = 1000,
	.speed.out_max = 20000,
	.angle.kp = 0.2,           //7
	.angle.ki = 0,
	.angle.kd = 0.2,
	.angle.integral_max = 0,
	.angle.out_max = 1680,
}; 
motor_pid_t GIMB_Y_gyro = {
	.speed.kp = 20,           //20
	.speed.ki = 0.15,         //0.15
	.speed.kd = 0,             //0
	.speed.integral_max = 1000,
	.speed.out_max = 20000,
	.angle.kp = 4,            // 7
	.angle.ki = 0,
	.angle.kd = 0,
	.angle.integral_max = 0,
	.angle.out_max = 1680,
};

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
	
//	Fric_B_UP.single_init(&Fric_B_UP);
//	Fric_F_UP.single_init(&Fric_F_UP);
//	Fric_B_R.single_init(&Fric_B_R);
//	Fric_B_L.single_init(&Fric_B_L);
//	Fric_F_R.single_init(&Fric_F_R);
	
	for(uint8_t i=0;i<MOTOR_MODE_LIST;i++)
	{
		gim_p_motor[i].single_init(&gim_p_motor[i]);
	}
	
//	Fric_F_L.single_init(&Fric_F_L);
	W_Dail.single_init(&W_Dail);
	
	RM_Group1.group_init(&RM_Group1);
	RM_Group2.group_init(&RM_Group2);
	
	

}

void kt_motor_list_init()
{
	kt_motor[0].init(&kt_motor[0]);
	
	motor_pid_init(&kt_motor[0].motor_all_pid.mec_pid,GIMB_Y_mec);
	motor_pid_init(&kt_motor[0].motor_all_pid.gyro_pid,GIMB_Y_gyro);
	
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
//	RM_Group.group_heartbeat(&RM_Group);
}


