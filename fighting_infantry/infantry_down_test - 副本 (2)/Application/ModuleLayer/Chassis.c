/**
  ******************************************************************************
  * File Name          : Chassis.c
  * Description        : Code for Chassis applications
  ******************************************************************************
  * @attention
  * Copyright (c) 2026 SZU RobotPilots.
  * @version
  * V1.0 October 2025 
  * @author
  * Liang 741427745@qq.com
  ******************************************************************************
  * 
  ==============================================================================
                      ##### How To Use #####
  ==============================================================================
  (#) 调用软件层初始化函数 Chassis_Init
  
  (#) 调用底盘电机心跳函数 Chassis_HeartBeat
  
  (#) 调用 Chassis_Status_React ，根据车模式来响应底盘状态，依赖Balance文件
		
  (#) 调用 Chassis_Data_Update ，包含整车状态更新和获取遥控器数据的更新
  
  (#) 调用 Chassis_Ctrl ，根据底盘状态来进行相应控制
  
  ==============================================================================
                      ##### How To Input Data #####
  ============================================================================== 
  (#)   车体方向定义：从车的前进方向的右边来观测连杆；无论是左腿和右腿，观测方向都不变。
		例如有一个人站在你面前，你走到他的右手边来观测他的手的运动。
  (#)   以下为正方向：
  (#)	基于HGC模型:
		①https://zhuanlan.zhihu.com/p/563048952
		②https://zhuanlan.zhihu.com/p/613007726
		
  (#) Chassis_Posture 里输出的数据方向应该：
		以你的头为本体，来示意方向
		pitch	方向：抬头为正
		roll	方向：头往右边歪为正
		yaw 	方向：头往左扭为正
		
  (#) Chassis_Data_Update 里需要保证以下数据输入正确：
		phi1 						 单位：rad  ，方向：逆时针,零点：水平向左
		phi4 						 单位：rad  ，方向：逆时针，零点：水平向左
		phi1_d1 					 单位：rad/s  ，方向：逆时针
		phi4_d1 					 单位：rad/s  ，方向：逆时针
		torque_phi1_mea				 单位：N*m ，方向逆时针
		torque_phi4_mea				 单位：N*m ，方向逆时针
		
		
  (#) Chassis_State_Var_Update 里需要保证以下数据输入正确：
		vir_phi0 					 单位：rad  ，方向：逆时针 ，零点：水平向下
		pitch 						 单位：rad  ，方向：逆时针 ，零点：水平向右
		vir_phi0_d1 				 单位：rad/s  ，方向：逆时针
		pitch_d1 					 单位：rad/s  ，方向：逆时针
		L_Wheel.rx_info->speed		 单位：rad/s  ，方向：顺时针
		R_Wheel.rx_info->speed		 单位：rad/s  ，方向：顺时针
		L_Wheel.rx_info->motor_angle_sum		 单位：rad  ，方向：顺时针
		R_Wheel.rx_info->motor_angle_sum		 单位：rad  ，方向：顺时针
		l0							 单位：m  ，方向：伸腿增大
		α							 -vir_phi0
		
		计算出来的变量方向：
		thetal				 		 单位：rad ，方向:顺时针，零点：水平向下
		thetal_d1					 单位：rad/s ，方向:顺时针
		stator_bias			    	 伸腿时相当于轮子后退（逆时针），具体请看Stator_Correction_Cal
		L_Wheel_speed_Transformed      单位：rad 方向：车前进时为正（顺时针）                                                                                                                                     
		R_Wheel_speed_Transformed	   单位：rad 方向：车前进时为正（顺时针）
		L_Wheel_anglesum_Transformed   单位：rad 方向：车前进时为正（顺时针）                                                                                                                                                    
		R_Wheel_anglesum_Transformed   单位：rad 方向：车前进时为正（顺时针）
		
	(#) Chassis_Target_Update 这里的方向自己定
	
	==============================================================================
								##### Quick Start #####
	============================================================================== 
  (#)	更改car_info的参数，使其符合车体建模
  
  (#)	连上关节电机，检查五连杆解算，l0，vir_phi0，及其导数是否正确
	
  (#)   连上轮毂电机，检查speed，anglesum是否正确
	
  (#)   连上板子，检查姿态输入是否正确
  
  (#)   检查s，sd1状态变量是否正确，检查状态变量误差是否正确，检查LQR能否正常输出Tp,Tw
  
  (#)   在Balance文件定义的 LEG_TEST_Mode 下检查 l0，phi0控制
 
  (#)	姿态差自动停止控制
  
  (#)	检查重力前馈
  
  (#)   定腿长调k矩阵，实现平衡，仅保留LQR，重力前馈。初始目标腿长在car_info里，k矩阵K_MATRIX_COEFFICIENT
	
  (#)	离地检测，仅保持thetal
	
  (#)	前进后退，旋转；需要测试双腿协调sync
  
  (#)	roll轴控制
  
  (#)	检查是否能防打滑
  
  (#)	根据腿长拟合k矩阵，实现平衡
  
  (#)	磕膝上台阶                                                               
  
  (#)	跳跃  


	==============================================================================
								##### Complete not yet #####
	============================================================================== 
	
	(#)	功率限制未写
  
	(#) 氮气弹簧前馈未写
	
	(#)	串腿自救未试过，未完善
	
	(#)	跳跃未充分验证，仅供参考
	
	(#) 撞膝上台阶未充分验证，仅供参考
	
	
  */

#include "Chassis.h"
#include "Gimbal.h"
#include "Board_protocol.h" 
#include "Judge.h"
#include "Flag_Handle.h"
#include "Filter.h"
#include "judge.h" 
void Chassis_Init(Chassis_t* My_Chassis);
static void Chassis_HeartBeat(Chassis_t* My_Chassis);
/*任务调用函数 begin*/
static void Chassis_Data_Update(Chassis_t* My_Chassis);
static void Chassis_Status_React(Chassis_t *My_Chassis);
static void Chassis_Ctrl(Chassis_t *My_Chassis);
/*任务调用函数 end*/
static void Chassis_Work(Chassis_t* My_Chassis);

static float My_Phi1_Transform(Leg_e My_Leg_e, Motor_DM_t* my_motor);
static float My_Phi4_Transform(Leg_e My_Leg_e, Motor_DM_t* my_motor);
static void Chassis_State_Var_Update(Chassis_t* My_Chassis);
static void Stator_Correction_Cal(Chassis_t *My_Chassis);
static void Chassis_Offline_Process(Chassis_t* My_Chassis);

/* 目标设置函数begin */
static void Chassis_Rc_Input_Update(Chassis_t* My_Chassis);
static void My_Chassis_KEY_Input(void);
void Key_Change(void);
static void Chassis_sd1_Target_Update(Chassis_t* My_Chassis);
static void Chassis_Yaw_Target_Process_All(Chassis_t* My_Chassis);
static void Chassis_Leg_Length_Target_Process(Chassis_t* My_Chassis);
static void Chassis_Speed_Limit(Chassis_t* My_Chassis);
static void Chassis_Target_Update(Chassis_t* My_Chassis);
/* 目标设置函数end */

/*腿长相关力 begin*/
static void Chassis_Leg_Length_Strength_Cal(Chassis_t* My_Chassis);
static void Chassis_Roll_Control(Chassis_t* My_Chassis);
static void Chassis_Link_Feedforward_Cal(Chassis_t* My_Chassis);
static void Chassis_Leg_Fbl_Cal(Chassis_t* My_Chassis);

/*腿长相关力 end*/
//双腿协调
static void Chassis_Leg_Sync_Cal(Chassis_t* My_Chassis);
//用于自救
static void Chassis_Leg_vir_phi0_Cal(Chassis_t* My_Chassis);
static void Chassis_Leg_phi0d1_Cal(Chassis_t* My_Chassis);
static void Clean_Process(Chassis_t* My_Chassis);
static void Rescue_Target_Process(Chassis_t* My_Chassis);


static void Chassis_Wheel_Turn_Cal(Chassis_t* My_Chassis);

/*计算chassis所有电机力矩 end*/
static void Chassis_Torque_Cal(Chassis_t *My_Chassis);
/*计算chassis所有电机力矩 end*/

//由控制层到电机层，将计算出来的扭矩赋给相应电机
static void Chassis_Set_Torque(Chassis_t* My_Chassis);
//电机层目标扭矩设0
static void Chassis_Motor_Set_Sleep(Chassis_t* My_Chassis);
//关控阻尼卸力，保护机械限位
static void Chassis_Stop_Damping(Chassis_t* My_Chassis);
//离地检测
static void Chassis_Takeoff_Detect(Chassis_t* My_Chassis);
//跳跃过程处理
static void Jump_Target_Process(Chassis_t* My_Chassis);
//撞膝上台阶
static void Knee_Strike_Target_Process(Chassis_t* My_Chassis);
//阻尼卸力
static void Chassis_Damping_Sleep(Chassis_t *My_Chassis);
//保护关节限位
static void My_Sd_Position_Nonlinear_Fix(Chassis_t* My_Chassis);


/* 测试用功能 begin*/
static void Test_phi0_l0_Ctrl(Chassis_t *My_Chassis);
static void Test_Straight_Ctrl(Chassis_t *My_Chassis);
/* 测试用功能 end*/

static void Chassis_Init_Ctrl(Chassis_t* My_Chassis);

//static void Chassis_Motor_Target_Angle(Chassis_t* My_Chassis);
//static void My_Chassis_KEY_Input(void);
static void Chassis_Motor_Group_Offline_Check(Chassis_t* My_Chassis);


//底盘功率限制
static void Chassis_Power_Limit(Chassis_t* My_Chassis);
static float My_Chassis_Power_Limit(void);
//氮气弹簧动态前馈
void My_Spring_Former_Input_Cal(Link_info_t* R_Link,Link_info_t* L_Link);


float Chassis_S_Turn_sdl_update(Chassis_t* My_Chassis);


//将Instance导入
Leg_force_t Leg_force[Leg_Num];
Leg_Unit_t Leg_Unit[Leg_Num]=
{
	[R_Leg].Straight=&Straight_Leg[R_Leg],
	[R_Leg].Link=&Link[R_Leg],
	
	[L_Leg].Straight=&Straight_Leg[L_Leg],
	[L_Leg].Link=&Link[L_Leg],
	
	[R_Leg].force=&Leg_force[R_Leg],
	[L_Leg].force=&Leg_force[L_Leg],
};

Chassis_Rc_Input_t Chassis_Rc_Input;
Chassis_state_t Chassis_state;
Chassis_reset_state_t  Chassis_reset_state;
Chassis_Jump_t Chassis_Jump;
Chassis_Knee_Strike_t Chassis_Knee_Strike;
Chassis_Rescue_t Chassis_Rescue;
Chassis_pid_init_parament_t Chassis_pid_init_parament[Leg_Num];
Chassis_Target_t Chassis_Target = 
{
	.s = 0.f,
	.sd1 = 0.f,
	.yaw_v = 0.f,
	.yaw = 0.f,
	.roll = 0.f,
	.thetal_r=0.f,
	.thetal_l=0.f,
	.leg_length_l = TAR_LEG_LENGTH_INITIAL,
	.leg_length_r = TAR_LEG_LENGTH_INITIAL,
};

Chassis_t Chassis = {
	.Init = Chassis_Init,
};

/**
  * @brief  底盘软件层初始化
  * @param  Chassis_t* My_Chassis
  * @retval None
  */
void Chassis_Init(Chassis_t* My_Chassis)
{
	/*结构体初始化*/
	My_Chassis->Posture = &Chassis_Posture;
	My_Chassis->Wheel=&Wheel_Group;
	My_Chassis->Sd= &Sd_Group;
	My_Chassis->state = &Chassis_state;
	My_Chassis->reset_struct = &Chassis_reset_state;
	My_Chassis->rc_input = &Chassis_Rc_Input;
	My_Chassis->damping_delay_cnt = DAMPING_DELAY_MAX_CNT;
	My_Chassis->reset_struct->reset_cnt = 0;
	My_Chassis->target = &Chassis_Target;
	My_Chassis->chassis_PID=&chassis_PID;
	My_Chassis->jump_info=&Chassis_Jump;
	My_Chassis->knee_strike_info=&Chassis_Knee_Strike;
	My_Chassis->rescue_info=&Chassis_Rescue;
	My_Chassis->pid_init_parament[R_Leg]=&Chassis_pid_init_parament[R_Leg];
	My_Chassis->pid_init_parament[L_Leg]=&Chassis_pid_init_parament[L_Leg];
	My_Chassis->Leg_Unit[R_Leg]=&Leg_Unit[R_Leg];
	My_Chassis->Leg_Unit[L_Leg]=&Leg_Unit[L_Leg];
	/*参数初始化*/
	//跳跃
	Chassis_Jump.Minimum_l0_range=0.01f; //NO_PRE_LANDING
	Chassis_Jump.Max_l0_range=0.02f;		//NO_PRE_LANDING
	Chassis_Jump.Landing_l0_range=0.01f;
	
	Chassis_Jump.Max_COMPRESS_tick=500.f; //NO_PRE_LANDING
	Chassis_Jump.Max_EXTEND_tick=400.f;	//NO_PRE_LANDING
	Chassis_Jump.Max_RETRACT_tick=300.f;//700.f;	//NO_PRE_LANDING
	Chassis_Jump.Max_PRE_LANDING_tick=600.f;//300.f;
	Chassis_Jump.Max_LANDING_tick=1200.f;
	
	Chassis_Jump.COMPRESS_length_kp=400.f; //NO_PRE_LANDING
	Chassis_Jump.EXTEND_length_kp=4000.f;  //NO_PRE_LANDING
	Chassis_Jump.RETRACT_length_kp=4000.f; //NO_PRE_LANDING
	Chassis_Jump.PRE_LANDING_length_kp=50.f;
	Chassis_Jump.LANDING_length_kp=100.f;//200.f;
	Chassis_Jump.LANDING_length_speed_kp=200.f;//10000.f;
	//撞膝上台阶
	Chassis_Knee_Strike.Minimum_l0_range=0.01f;
	Chassis_Knee_Strike.Max_l0_range=0.02f;
	Chassis_Knee_Strike.Max_Stand_High_tick=10000;
	Chassis_Knee_Strike.STAND_length_kp = 5.f;
	Chassis_Knee_Strike.RETRACT_length_kp=3000.f;//3000.f;
	Chassis_Knee_Strike.thetal_threshold=20.f;
	Chassis_Knee_Strike.Max_RETRACT_tick=500;
	//自救
	Chassis_Rescue.yaw_cnt_max = 500;
	Chassis_Rescue.yaw_save_range = PI/4.5;
	
	//pid初始参数
	My_Chassis->pid_init_parament[R_Leg]->l0_length_kp=My_Chassis->chassis_PID->length_cal[R_Leg]->kp;
  My_Chassis->pid_init_parament[L_Leg]->l0_length_kp=My_Chassis->chassis_PID->length_cal[L_Leg]->kp;
	My_Chassis->pid_init_parament[R_Leg]->l0_length_speed_kp=My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp;
	My_Chassis->pid_init_parament[L_Leg]->l0_length_speed_kp=My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp;
	My_Chassis->pid_init_parament[R_Leg]->l0_length_outmax=My_Chassis->chassis_PID->length_cal[R_Leg]->out_max;
	My_Chassis->pid_init_parament[L_Leg]->l0_length_outmax=My_Chassis->chassis_PID->length_cal[L_Leg]->out_max;
	My_Chassis->pid_init_parament[R_Leg]->l0_length_speed_outmax=My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max;
	My_Chassis->pid_init_parament[L_Leg]->l0_length_speed_outmax=My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max;
	
	/*电机初始化*/
	My_Chassis->Wheel->group_init(My_Chassis->Wheel);
	My_Chassis->Sd->group_init(My_Chassis->Sd);
	
	/*底盘函数初始化*/
	My_Chassis->heartbeat = Chassis_HeartBeat;
	My_Chassis->data_update = Chassis_Data_Update;
	My_Chassis->status_react = Chassis_Status_React;
	My_Chassis->work = Chassis_Work;
	My_Chassis->ctrl = Chassis_Ctrl;
	
	/*五连杆初始化*/
	
	My_Chassis->Leg_Unit[R_Leg]->Link->init(My_Chassis->Leg_Unit[R_Leg]->Link);
	My_Chassis->Leg_Unit[L_Leg]->Link->init(My_Chassis->Leg_Unit[L_Leg]->Link);
	My_Chassis->Leg_Unit[R_Leg]->Straight->init(My_Chassis->Leg_Unit[R_Leg]->Straight);
	My_Chassis->Leg_Unit[L_Leg]->Straight->init(My_Chassis->Leg_Unit[L_Leg]->Straight);
	
	/*卡尔曼滤波器初始化*/
	xvEstimateKF_Init(&vaEstimateKF);
	XEstimateKF_Init(&XEstimateKF);
	
	My_Chassis->Leg_Unit[R_Leg]->force->F_jump = 0;
	My_Chassis->Leg_Unit[L_Leg]->force->F_jump = 0;
	
	My_Chassis->target->velocity_max = 2.4f;
}

/**
  * @brief  底盘心跳包
  * @param  Chassis_t* My_Chassis
  * @retval None
  */
static void Chassis_HeartBeat(Chassis_t* My_Chassis)
{
	My_Chassis->Wheel->group_heartbeat(My_Chassis->Wheel);
	My_Chassis->Sd->group_heartbeat(My_Chassis->Sd);

	Chassis_Motor_Group_Offline_Check(My_Chassis);
	
	if(My_Chassis->state->sd_state == DEV_ONLINE && My_Chassis->state->wheel_state == DEV_ONLINE)
	{
		Balance.Flag->Chassis_Online_Flag = true;
	}
	else
	{
		Balance.Flag->Chassis_Online_Flag = false;
	}

}

/**
  * @brief  底盘运行
  * @param  Chassis_t* My_Chassis
  * @retval None
  */
static void Chassis_Work(Chassis_t* My_Chassis)
{
	My_Chassis->status_react(My_Chassis);
	
	My_Chassis->data_update(My_Chassis);
	
	My_Chassis->ctrl(My_Chassis);
}

/**
  * @brief  底盘数据更新
  * @param  Chassis_t* My_Chassis
  * @retval None
  */
static void Chassis_Data_Update(Chassis_t* My_Chassis)
{
	/*更新原始数据*/
	My_Chassis->Posture->data_update(My_Chassis->Posture);
	/*更新五连杆测量数据*/
	float phi1 = My_Phi1_Transform(R_Leg, My_Chassis->Sd->motor[R_F_Sd_M]);
	float phi4 = My_Phi4_Transform(R_Leg, My_Chassis->Sd->motor[R_B_Sd_M]);
	float phi1_d1 = My_Chassis->Sd->motor[R_F_Sd_M]->rx_info->speed;
	float phi4_d1 = My_Chassis->Sd->motor[R_B_Sd_M]->rx_info->speed;
	float torque_phi1_mea=My_Chassis->Sd->motor[R_F_Sd_M]->rx_info->torque;
	float torque_phi4_mea=My_Chassis->Sd->motor[R_B_Sd_M]->rx_info->torque;
	My_Chassis->Leg_Unit[R_Leg]->Link->mea_data_update(My_Chassis->Leg_Unit[R_Leg]->Link,phi1,phi1_d1,phi4,phi4_d1,torque_phi1_mea,torque_phi4_mea);
	
	phi1 = My_Phi1_Transform(L_Leg, My_Chassis->Sd->motor[L_F_Sd_M]);
	phi4 = My_Phi4_Transform(L_Leg, My_Chassis->Sd->motor[L_B_Sd_M]);
	phi1_d1 = -My_Chassis->Sd->motor[L_F_Sd_M]->rx_info->speed;
	phi4_d1 = -My_Chassis->Sd->motor[L_B_Sd_M]->rx_info->speed;
	torque_phi1_mea= - My_Chassis->Sd->motor[L_F_Sd_M]->rx_info->torque;
	torque_phi4_mea= - My_Chassis->Sd->motor[L_B_Sd_M]->rx_info->torque;
	My_Chassis->Leg_Unit[L_Leg]->Link->mea_data_update(My_Chassis->Leg_Unit[L_Leg]->Link,phi1,phi1_d1,phi4,phi4_d1,torque_phi1_mea,torque_phi4_mea);

	
	/*更新五连杆数据*/
	My_Chassis->Leg_Unit[R_Leg]->Link->link_update(My_Chassis->Leg_Unit[R_Leg]->Link);
	My_Chassis->Leg_Unit[L_Leg]->Link->link_update(My_Chassis->Leg_Unit[L_Leg]->Link);
	
	
	//氮气弹簧在VMC逆解算前求出，将前馈力叠加到关节电机力矩上
	My_Spring_Former_Input_Cal(My_Chassis->Leg_Unit[R_Leg]->Link->info,My_Chassis->Leg_Unit[L_Leg]->Link->info);
	
	#ifndef SPRING_USED
	float R_T1 = My_Chassis->Sd->motor[R_F_Sd_M]->rx_info->torque;
	float R_T2 = My_Chassis->Sd->motor[R_B_Sd_M]->rx_info->torque;
	
	float L_T1 = My_Chassis->Sd->motor[L_F_Sd_M]->rx_info->torque;
	float L_T2 = My_Chassis->Sd->motor[L_B_Sd_M]->rx_info->torque;
	#else
	float R_T1 = My_Chassis->Sd->motor[R_F_Sd_M]->rx_info->torque - My_Chassis->Leg_Unit[R_Leg]->Link->info->force->Spring_T_Feed_Front;
	float R_T2 = My_Chassis->Sd->motor[R_B_Sd_M]->rx_info->torque + My_Chassis->Leg_Unit[R_Leg]->Link->info->force->Spring_T_Feed_Back;
	 
	float L_T1 = -(My_Chassis->Sd->motor[L_F_Sd_M]->rx_info->torque + My_Chassis->Leg_Unit[L_Leg]->Link->info->force->Spring_T_Feed_Front);
	float L_T2 = -(My_Chassis->Sd->motor[L_B_Sd_M]->rx_info->torque - My_Chassis->Leg_Unit[L_Leg]->Link->info->force->Spring_T_Feed_Back);
	#endif
	                                                                                 

	/*VMC逆解算*/
	My_Chassis->Leg_Unit[R_Leg]->Link->Fb1_Tp_cal(My_Chassis->Leg_Unit[R_Leg]->Link,R_T1,R_T2);
	My_Chassis->Leg_Unit[L_Leg]->Link->Fb1_Tp_cal(My_Chassis->Leg_Unit[L_Leg]->Link,L_T1,L_T2);
	
	/*输入到直腿模型*/
	My_Chassis->Leg_Unit[R_Leg]->Straight->ex_data_update(My_Chassis->Leg_Unit[R_Leg]->Straight,My_Chassis->Leg_Unit[R_Leg]->Link->info->length->l0);
	My_Chassis->Leg_Unit[L_Leg]->Straight->ex_data_update(My_Chassis->Leg_Unit[L_Leg]->Straight,My_Chassis->Leg_Unit[L_Leg]->Link->info->length->l0);
	
	/*K矩阵更新*/
	#ifndef NO_K_Fitting
	My_Chassis->Leg_Unit[R_Leg]->Straight->K_fitting(My_Chassis->Leg_Unit[R_Leg]->Straight);
	My_Chassis->Leg_Unit[L_Leg]->Straight->K_fitting(My_Chassis->Leg_Unit[L_Leg]->Straight);
	#endif
	
	 
	
	/*状态量更新，直接对Straight结构体操作*/
	Chassis_State_Var_Update(My_Chassis);
	
	/*目标值更新*/
	Chassis_Target_Update(My_Chassis);//放在Chassis_State_Var_Update后面，才好处理离地情况

}


/**
  * @brief  底盘目标值更新
  * @param  None
  * @retval None
  */
static void Chassis_Target_Update(Chassis_t* My_Chassis)
{
	Chassis_Rc_Input_Update(My_Chassis);//遥控器输入值步进限幅滤波
//	Key_Change();
	My_Chassis_KEY_Input();
	Chassis_sd1_Target_Update(My_Chassis);//控sd1
	Chassis_Yaw_Target_Process_All(My_Chassis);//输出My_Chassis->target->yaw
	Chassis_Leg_Length_Target_Process(My_Chassis);//腿长目标值控制
	Chassis_Speed_Limit(My_Chassis);//sd1目标值限制
	
	/*离地时位移和pitch的目标值=状态量，使这两项输出为0*/
	if(My_Chassis->Leg_Unit[R_Leg]->off_ground == true)
	{
		My_Chassis->Leg_Unit[R_Leg]->Straight->target_state_update(My_Chassis->Leg_Unit[R_Leg]->Straight,
												My_Chassis->target->thetal_r,My_Chassis->target->thetald1,
												My_Chassis->Leg_Unit[R_Leg]->Straight->info->s,My_Chassis->Leg_Unit[R_Leg]->Straight->info->sd1,
												My_Chassis->Leg_Unit[R_Leg]->Straight->info->thetab,My_Chassis->Leg_Unit[R_Leg]->Straight->info->thetabd1) ;
	}
	else
	{
		My_Chassis->Leg_Unit[R_Leg]->Straight->target_state_update(My_Chassis->Leg_Unit[R_Leg]->Straight,
												My_Chassis->target->thetal_r,My_Chassis->target->thetald1,
												My_Chassis->target->s,My_Chassis->target->sd1,
												My_Chassis->target->thetab,My_Chassis->target->thetabd1) ;
	}
	if(My_Chassis->Leg_Unit[L_Leg]->off_ground == true)
	{
		My_Chassis->Leg_Unit[L_Leg]->Straight->target_state_update(My_Chassis->Leg_Unit[L_Leg]->Straight,
												My_Chassis->target->thetal_r,My_Chassis->target->thetald1,
												My_Chassis->Leg_Unit[L_Leg]->Straight->info->s,My_Chassis->Leg_Unit[L_Leg]->Straight->info->sd1,
												My_Chassis->Leg_Unit[L_Leg]->Straight->info->thetab,My_Chassis->Leg_Unit[L_Leg]->Straight->info->thetabd1) ;
	}
	else
	{
		My_Chassis->Leg_Unit[L_Leg]->Straight->target_state_update(My_Chassis->Leg_Unit[L_Leg]->Straight,
												My_Chassis->target->thetal_r,My_Chassis->target->thetald1,
												My_Chassis->target->s,My_Chassis->target->sd1,
												My_Chassis->target->thetab,My_Chassis->target->thetabd1) ;
	}
	
}

/**
  * @brief  状态变量更新
  * @param  Chassis_t* My_Chassis
  * @retval None
  */
static void Chassis_State_Var_Update(Chassis_t* My_Chassis)//角度均用弧度制
{
	/*结构体输入*/
	State_info_t* R_Leg_State_Var = My_Chassis->Leg_Unit[R_Leg]->Straight->info;
	State_info_t* L_Leg_State_Var = My_Chassis->Leg_Unit[L_Leg]->Straight->info;
	Link_t * R_Leg_Link =My_Chassis->Leg_Unit[R_Leg]->Link;
	Link_t * L_Leg_Link =My_Chassis->Leg_Unit[L_Leg]->Link;
	Chassis_Posture_info_t* My_Posture = My_Chassis->Posture->info;
	Motor_RM_t L_Wheel = *My_Chassis->Wheel->motor[L_WHEEL_M];
	Motor_RM_t R_Wheel = *My_Chassis->Wheel->motor[R_WHEEL_M];
	
	/*杆倾斜角度、机体角度 begin*/
	//与上交模型不同
	R_Leg_State_Var->thetal_last=R_Leg_State_Var->thetal;
	R_Leg_State_Var->thetal = (R_VIR_PHI0_ORDER_CORRECT*R_Leg_Link->info->angle->vir_phi0 - My_Posture->pitch);
	R_Leg_State_Var->thetald1 = R_VIR_PHI0_ORDER_CORRECT*R_Leg_Link->info->angle->vir_phi0_d1-My_Posture->pitch_v;
	R_Leg_State_Var->thetald1_l_now=R_Leg_State_Var->thetald1;
	R_Leg_State_Var->thetald2=R_Leg_State_Var->thetald1_l_now-R_Leg_State_Var->thetald1_l_last;
	R_Leg_State_Var->thetald1_l_last=R_Leg_State_Var->thetald1_l_now;
	R_Leg_State_Var->thetab= My_Posture->pitch;
	R_Leg_State_Var->thetabd1= My_Posture->pitch_v;
	
	L_Leg_State_Var->thetal_last=L_Leg_State_Var->thetal;
	L_Leg_State_Var->thetal = (L_VIR_PHI0_ORDER_CORRECT*L_Leg_Link->info->angle->vir_phi0 - My_Posture->pitch);
	L_Leg_State_Var->thetald1 = L_VIR_PHI0_ORDER_CORRECT*L_Leg_Link->info->angle->vir_phi0_d1-My_Posture->pitch_v;
	L_Leg_State_Var->thetald1_l_now=L_Leg_State_Var->thetald1;
	L_Leg_State_Var->thetald2=L_Leg_State_Var->thetald1_l_now-L_Leg_State_Var->thetald1_l_last;
	L_Leg_State_Var->thetald1_l_last=L_Leg_State_Var->thetald1_l_now;
	L_Leg_State_Var->thetab= My_Posture->pitch;
	L_Leg_State_Var->thetabd1= My_Posture->pitch_v;
	/*杆倾斜角度、机体角度 end*/
	
	
	
	
	/*消除杆动-->电机定子动-->编码器变化带来的影响*/
//	Stator_Correction_Cal(My_Chassis);
	
	/*俯仰角与俯仰角速度 begin*/
  R_Leg_State_Var->thetab = My_Posture->pitch;
  L_Leg_State_Var->thetab = My_Posture->pitch;
  R_Leg_State_Var->thetabd1 = My_Posture->pitch_v;
  L_Leg_State_Var->thetabd1 = My_Posture->pitch_v;
	/*俯仰角与俯仰角速度 end*/
	
	/*卡尔曼滤波更新*/
	/*路程与速度 begin*/
	float My_Wheel_Sb;
	float My_Imu_Sb;
	float My_filter_Sb;

	float My_filter_S;
	float My_filter_S_d1;
	
	float L_Wheel_speed_Transformed= L_W_SPEED_ORDER_CORRECT* L_Wheel.rx_info->speed;                                                                                                                                                             
	float R_Wheel_speed_Transformed= R_W_SPEED_ORDER_CORRECT*R_Wheel.rx_info->speed;
	float L_Wheel_anglesum_Transformed= L_Wheel.rx_info->motor_angle_sum;//已在chassis_motor矫正方向                                                                                                                                                             
	float R_Wheel_anglesum_Transformed= R_Wheel.rx_info->motor_angle_sum;
	//轮速+腿速
	My_Wheel_Sb = (WHEEL_RADIUS*(L_Wheel_speed_Transformed+R_Wheel_speed_Transformed) \
	               + (R_Leg_Link->info->length->l0*arm_cos_f32(R_Leg_State_Var->thetal)*R_Leg_State_Var->thetald1) \
	               + (L_Leg_Link->info->length->l0*arm_cos_f32(L_Leg_State_Var->thetal)*L_Leg_State_Var->thetald1))/2.f;
	My_Imu_Sb = My_Chassis->Posture->info->x_world;//机体加速度
	xvEstimateKF_Update(&vaEstimateKF, My_Imu_Sb, My_Wheel_Sb);
	My_filter_Sb  = vaEstimateKF.FilteredValue[0];//轮速+腿速

	
	//轮子位移（已考虑定子转动）
	float s = 0.5f * WHEEL_RADIUS * ((float)L_Wheel_anglesum_Transformed -L_Leg_Link->info->stator_correction->stator_bias \
	                                         + (float)R_Wheel_anglesum_Transformed - R_Leg_Link->info->stator_correction->stator_bias);
		
	//轮速
	float sd1 =My_filter_Sb -
	                   (((L_Leg_Link->info->length->l0*arm_cos_f32(L_Leg_State_Var->thetal)*L_Leg_State_Var->thetald1) \
	                    +(R_Leg_Link->info->length->l0*arm_cos_f32(R_Leg_State_Var->thetal)*R_Leg_State_Var->thetald1))/2.f);
	
//	My_State_Var->sd1 = 0.5f * WHEEL_RADIUS * (-(float)L_Wheel.rx_info->speed \
		                                       + (float)R_Wheel.rx_info->speed);
	XEstimateKF_Update(&XEstimateKF, sd1, s);
	My_filter_S = XEstimateKF.FilteredValue[0];//位移
	My_filter_S_d1 =  XEstimateKF.FilteredValue[1];//速度
	
	R_Leg_State_Var->s = My_filter_S;
	L_Leg_State_Var->s = My_filter_S;
	
	R_Leg_State_Var->sdl_last = R_Leg_State_Var->sd1;
	L_Leg_State_Var->sdl_last = L_Leg_State_Var->sd1;
	
	R_Leg_State_Var->sd1 = My_filter_S_d1;
	L_Leg_State_Var->sd1 = My_filter_S_d1;
	
	R_Leg_State_Var->sdl_now = R_Leg_State_Var->sd1;
	L_Leg_State_Var->sdl_now = L_Leg_State_Var->sd1;
	
	
	/*路程与速度 end*/
}
static void Test_Straight_Ctrl(Chassis_t *My_Chassis)
{
	Link_t* My_L_Link = My_Chassis->Leg_Unit[L_Leg]->Link;
	Link_t* My_R_Link = My_Chassis->Leg_Unit[R_Leg]->Link;
	Straight_Leg_t* R_Straight = My_Chassis->Leg_Unit[R_Leg]->Straight;
	Straight_Leg_t* L_Straight = My_Chassis->Leg_Unit[L_Leg]->Straight;
	/*直腿模型计算，得到驱动轮输出力矩和虚拟关节力矩*/

	/*-----------求Tp_target begin--------*/
	
	Chassis_Leg_Sync_Cal(My_Chassis);
	
	R_Straight->LQR_cal(R_Straight);//在目标值处做离地处理
	L_Straight->LQR_cal(L_Straight);
	My_Chassis->Leg_Unit[R_Leg]->force->Tp_LQR=R_Straight->get_Tp(R_Straight);
	My_Chassis->Leg_Unit[L_Leg]->force->Tp_LQR=L_Straight->get_Tp(L_Straight);

	 if (My_Chassis->Leg_Unit[R_Leg]->off_ground == true||
		 My_Chassis->Leg_Unit[L_Leg]->off_ground == true) //离地处理
    {
        My_Chassis->Leg_Unit[R_Leg]->force->Tp_target= R_TP_LQR_ORDER_CORRECT* My_Chassis->Leg_Unit[R_Leg]->force->Tp_LQR
													+My_Chassis->Leg_Unit[R_Leg]->force->Tp_sync;
		My_Chassis->Leg_Unit[L_Leg]->force->Tp_target= L_TP_LQR_ORDER_CORRECT* My_Chassis->Leg_Unit[L_Leg]->force->Tp_LQR
													+My_Chassis->Leg_Unit[L_Leg]->force->Tp_sync;
    }
	else{
		My_Chassis->Leg_Unit[R_Leg]->force->Tp_target= R_TP_LQR_ORDER_CORRECT* My_Chassis->Leg_Unit[R_Leg]->force->Tp_LQR;
		My_Chassis->Leg_Unit[L_Leg]->force->Tp_target= L_TP_LQR_ORDER_CORRECT* My_Chassis->Leg_Unit[L_Leg]->force->Tp_LQR;
	}
	
	
	
	/*-----------求Tp_target end-----------*/
	
	/*-----------求Fb1_target begin--------*/
	/*腿长控制力计算*/
	Chassis_Leg_Length_Strength_Cal(My_Chassis);
	
	/*roll控制力计算*/
	Chassis_Roll_Control(My_Chassis);
	
	/*前馈计算*/
	Chassis_Link_Feedforward_Cal(My_Chassis);
	
	/*汇总得到Fbl_target*/
	My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target =	  My_Chassis->Leg_Unit[R_Leg]->force->F
														+ My_Chassis->Leg_Unit[R_Leg]->force->F_gravity
														+ My_Chassis->Leg_Unit[R_Leg]->force->F_roll
														+ My_Chassis->Leg_Unit[R_Leg]->force->F_inertial;
	My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target =	  My_Chassis->Leg_Unit[L_Leg]->force->F
														+ My_Chassis->Leg_Unit[L_Leg]->force->F_gravity
														+ My_Chassis->Leg_Unit[L_Leg]->force->F_roll
														+ My_Chassis->Leg_Unit[L_Leg]->force->F_inertial;
	
	/*-----------求Fb1_target end--------*/
	
	/*-----------求Tw_target begin--------*/
	/*驱动轮转向环Tw_turn*/
	Chassis_Wheel_Turn_Cal(My_Chassis);
	My_Chassis->Leg_Unit[R_Leg]->force->Tw_LQR=R_Straight->get_Tw(R_Straight);
	My_Chassis->Leg_Unit[L_Leg]->force->Tw_LQR=L_Straight->get_Tw(L_Straight);
	/* 驱动轮电机最终输出 */
	 if (My_Chassis->Leg_Unit[R_Leg]->off_ground == true) // 离地处理
    {
        My_Chassis->Leg_Unit[R_Leg]->force->Tw_target = 0;
    }
    else
    {
        My_Chassis->Leg_Unit[R_Leg]->force->Tw_target = My_Chassis->Leg_Unit[R_Leg]->force->Tw_LQR 
														+ My_Chassis->Leg_Unit[R_Leg]->force->Tw_turn;
    }
    if (My_Chassis->Leg_Unit[L_Leg]->off_ground == true) // 离地处理
    {
        My_Chassis->Leg_Unit[L_Leg]->force->Tw_target = 0;
    }
    else
    {
        My_Chassis->Leg_Unit[L_Leg]->force->Tw_target = My_Chassis->Leg_Unit[L_Leg]->force->Tw_LQR 
														+ My_Chassis->Leg_Unit[L_Leg]->force->Tw_turn;
    }
	/*-----------求Tw_target end--------*/
	
	My_R_Link->tar_data_update(My_R_Link,My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target,My_Chassis->Leg_Unit[R_Leg]->force->Tp_target);
	My_L_Link->tar_data_update(My_L_Link,My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target,My_Chassis->Leg_Unit[L_Leg]->force->Tp_target);
	
	/*转换为关节力矩,输出到Link结构体的F_Sd_Output_Torque，B_Sd_Output_Torque*/
	My_L_Link->torque_cal(My_L_Link);
	My_R_Link->torque_cal(My_R_Link);
//	/*限位力矩补偿*/
//	My_Sd_Position_Nonlinear_Fix(My_Chassis);
	/* 关节电机最终输出 */
//	My_Chassis->Leg_Unit[R_Leg]->force->Sd_F_Torque=My_R_Link->info->F_Sd_Output_Torque+My_Chassis->Leg_Unit[R_Leg]->force->Sd_F_Limit_Tor_Fix;
//	My_Chassis->Leg_Unit[R_Leg]->force->Sd_B_Torque=My_R_Link->info->B_Sd_Output_Torque+My_Chassis->Leg_Unit[R_Leg]->force->Sd_B_Limit_Tor_Fix;
//	My_Chassis->Leg_Unit[L_Leg]->force->Sd_F_Torque=My_L_Link->info->F_Sd_Output_Torque+My_Chassis->Leg_Unit[L_Leg]->force->Sd_F_Limit_Tor_Fix;
//	My_Chassis->Leg_Unit[L_Leg]->force->Sd_B_Torque=My_L_Link->info->B_Sd_Output_Torque+My_Chassis->Leg_Unit[L_Leg]->force->Sd_B_Limit_Tor_Fix;

  
}

/**
  * @brief  控phi0、l0,把车架起来测,先把轮子卸力，两边最好单独测
  * @param  Chassis_t* My_Chassis
  * @retval None         
  */
static void Test_phi0_l0_Ctrl(Chassis_t *My_Chassis)
{
	Link_t* My_L_Link = My_Chassis->Leg_Unit[L_Leg]->Link;
	Link_t* My_R_Link = My_Chassis->Leg_Unit[R_Leg]->Link;
	/*直腿模型计算，得到驱动轮输出力矩和虚拟关节力矩*/

	/*-----------求Tp_target begin--------*/
	
	My_Chassis->target->vir_phi0_r +=My_Chassis->rc_input->ch3_now/660.f*TIME_STEP * 100;
	My_Chassis->target->vir_phi0_l +=My_Chassis->rc_input->ch3_now/660.f*TIME_STEP * 100;
//	My_Chassis->target->vir_phi0_r=constrain(My_Chassis->target->vir_phi0_r,angle2rad(-60),angle2rad(60));
//	My_Chassis->target->vir_phi0_l=constrain(My_Chassis->target->vir_phi0_l,angle2rad(-60),angle2rad(60));
	
	if(fabs(My_Chassis->target->vir_phi0_r)>= 180)
	{
		My_Chassis->target->vir_phi0_r -= sgn(My_Chassis->target->vir_phi0_r) * 360;
	}
	if(fabs(My_Chassis->target->vir_phi0_l)>= 180)
	{
		My_Chassis->target->vir_phi0_l -= sgn(My_Chassis->target->vir_phi0_l) * 360;
	}
	
	Chassis_Leg_vir_phi0_Cal(My_Chassis);//内部赋值给chassis
	
	My_Chassis->Leg_Unit[R_Leg]->force->Tp_target=My_Chassis->Leg_Unit[R_Leg]->force->Tp_vir_phi0_;
	My_Chassis->Leg_Unit[L_Leg]->force->Tp_target=My_Chassis->Leg_Unit[L_Leg]->force->Tp_vir_phi0_;
	/*-----------求Tp_target end--------*/
	
	/*-----------求Fb1_target begin--------*/

	/*腿长控制力计算*/
	
	Chassis_Leg_Length_Strength_Cal(My_Chassis);
//	
//	/*腿部前馈*/
//	//车架起来的时候前馈应该收腿
//	My_Chassis->Leg_Unit[R_Leg]->force->F_gravity = -( My_R_Link->info->centroid->centriod_coefficient*m_l) * g * cos(My_R_Link->info->angle->vir_phi0);
//	My_Chassis->Leg_Unit[L_Leg]->force->F_gravity = -( My_L_Link->info->centroid->centriod_coefficient*m_l) * g * cos(My_L_Link->info->angle->vir_phi0);
	
	My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target =	  My_Chassis->Leg_Unit[R_Leg]->force->F;
	
//														+ My_Chassis->Leg_Unit[R_Leg]->force->F_gravity;
//	My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target =	  My_Chassis->Leg_Unit[R_Leg]->force->F;							
	My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target =	  My_Chassis->Leg_Unit[L_Leg]->force->F;
//														+ My_Chassis->Leg_Unit[L_Leg]->force->F_gravity;

	/*-----------求Fb1_target end--------*/
	
	/*建模的Tp方向是顺时针，VMC是逆时针，所以从建模--->VMC要加个负号*/
	My_R_Link->tar_data_update(My_R_Link,My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target,My_Chassis->Leg_Unit[R_Leg]->force->Tp_target);
	My_L_Link->tar_data_update(My_L_Link,My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target,My_Chassis->Leg_Unit[L_Leg]->force->Tp_target);
	
	/*转换为关节力矩,输出到Link结构体的F_Sd_Output_Torque，B_Sd_Output_Torque*/
	
	My_L_Link->torque_cal(My_L_Link);
	My_R_Link->torque_cal(My_R_Link);
	
	/*赋值到chassis目标结构体*/
	My_Chassis->Leg_Unit[R_Leg]->force->Sd_F_Torque=My_R_Link->info->F_Sd_Output_Torque;
	My_Chassis->Leg_Unit[R_Leg]->force->Sd_B_Torque=My_R_Link->info->B_Sd_Output_Torque;
	My_Chassis->Leg_Unit[L_Leg]->force->Sd_F_Torque=My_L_Link->info->F_Sd_Output_Torque;
	My_Chassis->Leg_Unit[L_Leg]->force->Sd_B_Torque=My_L_Link->info->B_Sd_Output_Torque;
	

}
/**
  * @brief  底盘工作模式更新
  * @param  Chassis_t* My_Chassis
  * @retval None         
  */
static void Chassis_Status_React(Chassis_t *My_Chassis)
{
	switch(Balance.mode)
	{
		case Init_Mode:
			My_Chassis->mode = C_Init;
			break;
		case Sleep_Mode:
			My_Chassis->mode = C_Sleep;
		  
			break;
		case Imu_Mode:
			My_Chassis->mode = C_Follow;
//			if(Balance.command->slave->Slave_Online_Flag == false)
//				My_Chassis->mode = C_Boss;
			break;
		case Mec_Mode:
			My_Chassis->mode = C_Boss;
			break;
		
		case Turn_Mode:
			My_Chassis->mode = C_Turn;
      break;
		case Test_Mode:
			My_Chassis->mode = C_Test;
			break;
		
		case Sos_Mode:
			My_Chassis->mode = C_Rescue;
		  break;
		
		default:
			My_Chassis->mode = C_Sleep;
			break;
	}
	
  if(Balance.Flag->Jumping_Flag == true)	
	{
		My_Chassis->mode = C_Jump;
	}
	
	if(Balance.Flag->Knee_Strike_Flag == true)
	{
		My_Chassis->mode = C_Knee_Strike;
	}
	 
  if(Balance.Flag->Fly_Flag == true || Balance.Flag->Reserve_Fly_Flag == true)
  {
		My_Chassis->mode = C_Fly;
	}		
	
	
		//自救检测
		#ifndef NO_RESCUE
		if(Balance.Flag->Rescue_Flag == true)
		{
			My_Chassis->mode = C_Rescue;
			Balance.mode = Sos_Mode;

		}
		
		#endif
		
		
		
		
		
	//离线保护
	if(My_Chassis->state->sd_state == DEV_OFFLINE || 
		My_Chassis->state->wheel_state == DEV_OFFLINE||
			Balance.mode==Sleep_Mode
		)
	{
		My_Chassis->mode = C_Sleep;
	}
	
	if(My_Chassis->mode != C_Init)
		My_Chassis->reset_struct->reset_cnt = 0;
	
	if(Balance.Flag->Chassis_Sleep_Flag == true)
	{
		My_Chassis->mode = C_Sleep;
	}
}

/**
  * @brief  底盘阻尼卸力
  * @param  Chassis_t* My_Chassis
  * @retval None         
  */
static void Chassis_Damping_Sleep(Chassis_t *My_Chassis)
{
		if(Balance.Flag->Chassis_Online_Flag == false)
		{
			My_Chassis->Sd->group_sleep(My_Chassis->Sd);
			My_Chassis->Wheel->group_sleep(My_Chassis->Wheel);
		}
		else
		{
			if(My_Chassis->damping_delay_cnt < DAMPING_DELAY_MAX_CNT)
			{
				Chassis_Stop_Damping(My_Chassis);
				
				My_Chassis->damping_delay_cnt++; 
			}
			else
			{
				My_Chassis->Sd->group_sleep(My_Chassis->Sd);//含发送
				My_Chassis->Wheel->group_sleep(My_Chassis->Wheel);
			}
		}
}

/**
  * @brief  底盘总控制
  * @param  None
  * @retval None
  */
static void Chassis_Ctrl(Chassis_t *My_Chassis)
{
	Clean_Process(My_Chassis);
	
	switch(My_Chassis->mode)
	{
		case C_Sleep:
		Chassis_Takeoff_Detect(My_Chassis);
		Chassis_Torque_Cal(My_Chassis);
		Chassis_Damping_Sleep(My_Chassis);
		
		/*离线处理*/
		Chassis_Offline_Process(My_Chassis);
		break;
		
		case C_Boss:
		Chassis_Takeoff_Detect(My_Chassis);
		Chassis_Torque_Cal(My_Chassis);
		Chassis_Set_Torque(My_Chassis);
		break;
		
		case C_Follow:
		Chassis_Takeoff_Detect(My_Chassis);
		Chassis_Torque_Cal(My_Chassis);
		Chassis_Set_Torque(My_Chassis);
			break;
		
		case C_Turn:
		Chassis_Takeoff_Detect(My_Chassis);
		Chassis_Torque_Cal(My_Chassis);
		Chassis_Set_Torque(My_Chassis);
			break;
		
		case C_Init://不进行离地判断,不接受遥控和标志位响应
		Chassis_Torque_Cal(My_Chassis);
		Chassis_Set_Torque(My_Chassis);
		
		if(fabs(My_Chassis->Posture->info->pitch) <= 0.01f && My_Chassis->Leg_Unit[R_Leg]->Link->info->length->l0 - TAR_LEG_LENGTH_INITIAL <= 0.01f 
			                         && fabs(My_Chassis->Leg_Unit[L_Leg]->Link->info->length->l0 - TAR_LEG_LENGTH_INITIAL) <= 0.01f 
		                           && fabs(My_Chassis->Leg_Unit[L_Leg]->Straight->info->thetal) <= 0.01f 
		                           && fabs(My_Chassis->Leg_Unit[R_Leg]->Straight->info->thetal) <= 0.01f) 
		{
			My_Chassis->reset_struct->reset_state = Chassis_reset_OK;
		}
		else if(Balance.reset_struct.reset_state == Balance_reset_OK)
		{
			My_Chassis->reset_struct->reset_state = Chassis_reset_OK;
		}
		else
		{
			My_Chassis->reset_struct->reset_state = Chassis_reset_NO;
		}
		
		break;
		
		case C_Test:
	  #ifndef VISION_TEST
		  Chassis_Takeoff_Detect(My_Chassis);
		  
		  #ifndef FAKE_TEST
//		  Test_Straight_Ctrl(My_Chassis);
		  Test_phi0_l0_Ctrl(My_Chassis);
		  #else
		  Chassis_Torque_Cal(My_Chassis);
		  #endif
		  Chassis_Set_Torque(My_Chassis);
		
		#else
		 Chassis_Takeoff_Detect(My_Chassis);
		Chassis_Motor_Set_Sleep(My_Chassis);
		#endif
		break;
		
		case C_Rescue:
		   Rescue_Target_Process(My_Chassis);
		   Chassis_Set_Torque(My_Chassis);
		   break;
		
		case C_Jump:
			Chassis_Takeoff_Detect(My_Chassis);
	    Jump_Target_Process(My_Chassis);
		  Chassis_Torque_Cal(My_Chassis);
		  Chassis_Set_Torque(My_Chassis);
		  break;
		
		case C_Knee_Strike:
			 Chassis_Takeoff_Detect(My_Chassis);
		  Knee_Strike_Target_Process(My_Chassis);
		  Chassis_Torque_Cal(My_Chassis);
		  Chassis_Set_Torque(My_Chassis);
		  break;
		
		case C_Fly:
			Chassis_Takeoff_Detect(My_Chassis);
			Chassis_Torque_Cal(My_Chassis);
		  Chassis_Set_Torque(My_Chassis);
			break;
		
		default:
			My_Chassis->Sd->group_sleep(My_Chassis->Sd);//含发送
			My_Chassis->Wheel->group_sleep(My_Chassis->Wheel);
			Chassis_Offline_Process(My_Chassis);
			break;

		
	}
}
/**
  * @brief  清标志位等处理
  * @param  Chassis_t* My_Chassis, 底盘
  * @retval None
  */
static void Clean_Process(Chassis_t* My_Chassis)
{
	if(My_Chassis->mode!=C_Rescue)
	{
		My_Chassis->target->vir_phi0d1_r=0;
		My_Chassis->target->vir_phi0d1_l=0;
		
		My_Chassis->target->vir_phi0_l=0;
		My_Chassis->target->vir_phi0_r=0;
	}
	
	if(My_Chassis->mode!=C_Sleep)
	{
		My_Chassis->damping_delay_cnt = 0;
	}
	
}
/**
  * @brief  跳跃过程处理
  * @param  Chassis_t* My_Chassis, 底盘
  * @retval None
  */
static void Jump_Target_Process(Chassis_t* My_Chassis)
{
	//#define TIME_ONLY

	Chassis_Jump_t* jump_info=My_Chassis->jump_info;
	Link_t* R_Link=My_Chassis->Leg_Unit[R_Leg]->Link;
	Link_t* L_Link=My_Chassis->Leg_Unit[L_Leg]->Link;
	jump_info->r_offground=My_Chassis->Leg_Unit[R_Leg]->off_ground;
	jump_info->l_offground=My_Chassis->Leg_Unit[L_Leg]->off_ground;
	
	jump_info->l0_average = 0.5f*(R_Link->info->length->l0+L_Link->info->length->l0);
	
	switch (jump_info->jump_step)
	{
		case J_IDLE:
			//动作
			jump_info->IDLE_length_r_kp=My_Chassis->chassis_PID->length_cal[R_Leg]->kp;
			jump_info->IDLE_length_r_speed_kp=My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp;
			jump_info->IDLE_length_r_outmax=My_Chassis->chassis_PID->length_cal[R_Leg]->out_max;
			jump_info->IDLE_length_r_speed_outmax=My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max;
		
		  jump_info->IDLE_length_l_kp=My_Chassis->chassis_PID->length_cal[L_Leg]->kp;
			jump_info->IDLE_length_l_speed_kp=My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp;
			jump_info->IDLE_length_l_outmax=My_Chassis->chassis_PID->length_cal[L_Leg]->out_max;
			jump_info->IDLE_length_l_speed_outmax=My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max;
		  

			My_Chassis->chassis_PID->length_cal[R_Leg]->out_max=1000.f;
			My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max=1000.f;
			My_Chassis->chassis_PID->length_cal[L_Leg]->out_max=1000.f;
			My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max=1000.f;
			
			jump_info->COMPRESS_tick=0;
			jump_info->EXTEND_tick=0;
			jump_info->RETRACT_tick=0;
			jump_info->PRE_LANDING_tick=0;
			jump_info->LANDING_tick=0;
			//事件
			jump_info->jump_step=J_COMPRESS;
			break;
		
		case J_COMPRESS://压缩腿  
			//动作
			jump_info->COMPRESS_tick++;
			My_Chassis->target->leg_length_l = MIN_LEG_LENGTH+jump_info->Minimum_l0_range;
			My_Chassis->target->leg_length_r = MIN_LEG_LENGTH+jump_info->Minimum_l0_range;
			My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->COMPRESS_length_kp;
		    My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->COMPRESS_length_kp;
			//事件
			#ifdef TIME_ONLY
				if(jump_info->COMPRESS_tick>=jump_info->Max_COMPRESS_tick)
			jump_info->jump_step=J_EXTEND;
			#else
				if((jump_info->l0_average<=MIN_LEG_LENGTH+jump_info->Minimum_l0_range)
				|| jump_info->COMPRESS_tick>=jump_info->Max_COMPRESS_tick)
			jump_info->jump_step=J_EXTEND;
			#endif
			
			break;
			
		case J_EXTEND://跳
			//动作
			jump_info->EXTEND_tick++;
			My_Chassis->target->leg_length_l = MAX_LEG_LENGTH-jump_info->Max_l0_range;
			My_Chassis->target->leg_length_r = MAX_LEG_LENGTH-jump_info->Max_l0_range;
			My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->EXTEND_length_kp;
		  My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->EXTEND_length_kp;
		
		  My_Chassis->Leg_Unit[R_Leg]->force->F_jump = 280.f;
		  My_Chassis->Leg_Unit[L_Leg]->force->F_jump = 280.f;
			//事件
			#ifdef TIME_ONLY
			if(jump_info->EXTEND_tick>=jump_info->Max_EXTEND_tick)
			#else
				if((jump_info->l0_average>=MAX_LEG_LENGTH -jump_info->Max_l0_range)
				||jump_info->EXTEND_tick>=jump_info->Max_EXTEND_tick)
			#endif
			{	
			  jump_info->jump_step=J_RETRACT;
				
			  My_Chassis->Leg_Unit[R_Leg]->force->F_jump = 0.f;
		    My_Chassis->Leg_Unit[L_Leg]->force->F_jump = 0.f;
			}
			break;
		
		case J_RETRACT://收腿滞空
			//动作
			jump_info->RETRACT_tick++;
			My_Chassis->target->leg_length_l = MIN_LEG_LENGTH+jump_info->Minimum_l0_range;
			My_Chassis->target->leg_length_r = MIN_LEG_LENGTH+jump_info->Minimum_l0_range;
			My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->RETRACT_length_kp;
		    My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->RETRACT_length_kp;
			//事件
//			if(jump_info->r_offground==false&&jump_info->l_offground==false)
//			{
//				Balance.Flag->Jumping_Flag = false;
//				jump_info->jump_step=J_IDLE;
//				My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->IDLE_length_kp;
//				My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp=jump_info->IDLE_length_speed_kp;
//				My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->IDLE_length_kp;
//				My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp=jump_info->IDLE_length_speed_kp;
//			}	
			
			
				#ifdef NO_PRE_LANDING
				#ifdef TIME_ONLY
				if(jump_info->RETRACT_tick>=jump_info->Max_RETRACT_tick)
				#else
				if(jump_info->l0_average-(MIN_LEG_LENGTH+jump_info->Minimum_l0_range)<=0.f
					||jump_info->RETRACT_tick>=jump_info->Max_RETRACT_tick)
				#endif
				
				{
					Balance.Flag->Jumping_Flag = false;
					jump_info->jump_step=J_IDLE;
					My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->IDLE_length_kp;
					My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp=jump_info->IDLE_length_speed_kp;
					My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->IDLE_length_kp;
					My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp=jump_info->IDLE_length_speed_kp;
					My_Chassis->chassis_PID->length_cal[R_Leg]->out_max=jump_info->IDLE_length_outmax;
					My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max=jump_info->IDLE_length_speed_outmax;
					My_Chassis->chassis_PID->length_cal[L_Leg]->out_max=jump_info->IDLE_length_outmax;
					My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max=jump_info->IDLE_length_speed_outmax;
				}

				#else
				if(jump_info->RETRACT_tick>=jump_info->Max_RETRACT_tick)
				{
					jump_info->jump_step=J_PRE_LANDING;
				}
				#endif
			break;
		
		 case J_PRE_LANDING://伸腿准备缓冲
			//动作
			jump_info->PRE_LANDING_tick++;
			My_Chassis->target->leg_length_l += 0.004f;
			My_Chassis->target->leg_length_r += 0.004f;
			My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->PRE_LANDING_length_kp;
		    My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->PRE_LANDING_length_kp;
			//事件
		 
		 if(My_Chassis->target->leg_length_l >= MAX_LEG_LENGTH-jump_info->Max_l0_range)
		 {
			 My_Chassis->target->leg_length_l = MAX_LEG_LENGTH-jump_info->Max_l0_range;
		 }
		  if(My_Chassis->target->leg_length_r >= MAX_LEG_LENGTH-jump_info->Max_l0_range)
		 {
			 My_Chassis->target->leg_length_r = MAX_LEG_LENGTH-jump_info->Max_l0_range;
		 }
		 
			if(jump_info->r_offground==false||jump_info->l_offground==false)
			{
				if(My_Chassis->target->leg_length_l < TAR_LEG_LENGTH_INITIAL)
		    {
			    My_Chassis->target->leg_length_l = TAR_LEG_LENGTH_INITIAL + jump_info->Minimum_l0_range * 2;
		    }
		    if(My_Chassis->target->leg_length_r < TAR_LEG_LENGTH_INITIAL)
		    {
			    My_Chassis->target->leg_length_r = TAR_LEG_LENGTH_INITIAL + jump_info->Minimum_l0_range * 2; 
		    }
				
				jump_info->jump_step=J_LANDING;
			}	
			if(jump_info->PRE_LANDING_tick>=jump_info->Max_PRE_LANDING_tick)
			{
				if(My_Chassis->target->leg_length_l < TAR_LEG_LENGTH_INITIAL)
		    {
			    My_Chassis->target->leg_length_l = TAR_LEG_LENGTH_INITIAL;
		    }
		    if(My_Chassis->target->leg_length_r < TAR_LEG_LENGTH_INITIAL)
		    {
			    My_Chassis->target->leg_length_r = TAR_LEG_LENGTH_INITIAL;
		    }
				
				jump_info->jump_step=J_LANDING;
			}
			break;
			
		  case J_LANDING://缓冲
			//动作
			jump_info->LANDING_tick++;
			My_Chassis->target->leg_length_l -= 0.002f;
			My_Chassis->target->leg_length_r -= 0.002f;
			
			 if(My_Chassis->target->leg_length_l <= TAR_LEG_LENGTH_INITIAL)
		 {
			 My_Chassis->target->leg_length_l = TAR_LEG_LENGTH_INITIAL;
		 }
		  if(My_Chassis->target->leg_length_r <= TAR_LEG_LENGTH_INITIAL)
		 {
			 My_Chassis->target->leg_length_r = TAR_LEG_LENGTH_INITIAL;
		 }
			
			
			My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->LANDING_length_kp;
		    My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->LANDING_length_kp;
			My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp=jump_info->LANDING_length_speed_kp;
		    My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp=jump_info->LANDING_length_speed_kp;
			//事件
			if(jump_info->LANDING_tick>=jump_info->Max_LANDING_tick||
				(abs(jump_info->l0_average-TAR_LEG_LENGTH_INITIAL)<=jump_info->Landing_l0_range))
			{
				Balance.Flag->Jumping_Flag = false;
				jump_info->jump_step=J_IDLE;
				My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->IDLE_length_r_kp;
				My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp=jump_info->IDLE_length_r_speed_kp;
				My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->IDLE_length_r_kp;
				My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp=jump_info->IDLE_length_r_speed_kp;
				My_Chassis->chassis_PID->length_cal[R_Leg]->out_max=jump_info->IDLE_length_l_outmax;
				My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max=jump_info->IDLE_length_l_speed_outmax;
				My_Chassis->chassis_PID->length_cal[L_Leg]->out_max=jump_info->IDLE_length_l_outmax;
				My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max=jump_info->IDLE_length_l_speed_outmax;
			}
			break;
			
			
		  default:
            break;
	}
	
}


/**
  * @brief  撞膝上台阶
  * @param  Chassis_t* My_Chassis, 底盘
  * @retval None
  */
static void Knee_Strike_Target_Process(Chassis_t* My_Chassis)
{
	
	Chassis_Knee_Strike_t* knee_strike_info=My_Chassis->knee_strike_info;
	Link_t* R_Link=My_Chassis->Leg_Unit[R_Leg]->Link;
	Link_t* L_Link=My_Chassis->Leg_Unit[L_Leg]->Link;
	knee_strike_info->thetal_average=Rad2Angle*abs(0.5f*(My_Chassis->Leg_Unit[R_Leg]->Straight->info->thetal
										+My_Chassis->Leg_Unit[L_Leg]->Straight->info->thetal) - My_Chassis->Posture->info->pitch);
	
	knee_strike_info->l0_average = 0.5f*(R_Link->info->length->l0+L_Link->info->length->l0);
	
	switch (knee_strike_info->step)
	{
		case Knee_IDLE:
			//动作
			knee_strike_info->IDLE_length_r_kp=My_Chassis->chassis_PID->length_cal[R_Leg]->kp;//保存
		  knee_strike_info->IDLE_length_l_kp=My_Chassis->chassis_PID->length_cal[L_Leg]->kp;//保存
			knee_strike_info->Stand_High_tick=0;
			knee_strike_info->RETRACT_tick = 0;

			//事件
		  if(Balance.Flag->chassis_reset == false)
		  {
			  knee_strike_info->step=Knee_Stand_High;
	  	}
			
			break;
		
		case Knee_Stand_High://立着
			//动作
			knee_strike_info->Stand_High_tick++;
			My_Chassis->target->leg_length_l = MAX_LEG_LENGTH-knee_strike_info->Max_l0_range;
			My_Chassis->target->leg_length_r = MAX_LEG_LENGTH-knee_strike_info->Max_l0_range;
			
//		  if(knee_strike_info->l0_average >= 0.25f)
//			{
//				My_Chassis->chassis_PID->length_cal[R_Leg]->kp =knee_strike_info->STAND_length_kp;
//        My_Chassis->chassis_PID->length_cal[L_Leg]->kp =knee_strike_info->STAND_length_kp;
//				
//			} 
		
			//事件
			if(knee_strike_info->thetal_average>=knee_strike_info->thetal_threshold)
			{
				knee_strike_info->RETRACT_tick = 0;
				knee_strike_info->step=Knee_RETRACT;
			}
			if(knee_strike_info->Stand_High_tick>=knee_strike_info->Max_Stand_High_tick)
			{
				knee_strike_info->step=Knee_IDLE;
				My_Chassis->target->leg_length_l=TAR_LEG_LENGTH_INITIAL;
				My_Chassis->target->leg_length_r=TAR_LEG_LENGTH_INITIAL;
				My_Chassis->chassis_PID->length_cal[R_Leg]->kp= knee_strike_info->IDLE_length_r_kp;
				My_Chassis->chassis_PID->length_cal[L_Leg]->kp= knee_strike_info->IDLE_length_l_kp;
				Balance.Flag->Knee_Strike_Flag=false;
			}
			break;
			
		case Knee_RETRACT://收腿
			//动作
			knee_strike_info->RETRACT_tick++;
			My_Chassis->target->leg_length_l = MIN_LEG_LENGTH+knee_strike_info->Minimum_l0_range;
			My_Chassis->target->leg_length_r = MIN_LEG_LENGTH+knee_strike_info->Minimum_l0_range;
			My_Chassis->chassis_PID->length_cal[R_Leg]->kp=knee_strike_info->RETRACT_length_kp;
		    My_Chassis->chassis_PID->length_cal[L_Leg]->kp=knee_strike_info->RETRACT_length_kp;
			//事件
			if(knee_strike_info->RETRACT_tick>=knee_strike_info->Max_RETRACT_tick
				||knee_strike_info->l0_average<MIN_LEG_LENGTH+knee_strike_info->Minimum_l0_range)
			{
				knee_strike_info->step=Knee_IDLE;
				My_Chassis->target->leg_length_l=TAR_LEG_LENGTH_INITIAL;
				My_Chassis->target->leg_length_r=TAR_LEG_LENGTH_INITIAL;
				My_Chassis->chassis_PID->length_cal[R_Leg]->kp= knee_strike_info->IDLE_length_r_kp;
				My_Chassis->chassis_PID->length_cal[L_Leg]->kp= knee_strike_info->IDLE_length_l_kp;
				Balance.Flag->Knee_Strike_Flag=false;
			}
			
			break;
		
		  default:
            break;
	}
	
}
/**
  * @brief  自救目标值处理
  * @param  Chassis_t* My_Chassis, 底盘
  * @retval None
  */


static void Rescue_Target_Process(Chassis_t* My_Chassis)
{
	Chassis_Rescue_t* rescue_info=My_Chassis->rescue_info;
	
	Link_t* My_L_Link = My_Chassis->Leg_Unit[L_Leg]->Link;
	Link_t* My_R_Link = My_Chassis->Leg_Unit[R_Leg]->Link;
	Straight_Leg_t* My_L_Leg = My_Chassis->Leg_Unit[L_Leg]->Straight;
	Straight_Leg_t* My_R_Leg = My_Chassis->Leg_Unit[R_Leg]->Straight;
	
	static float R_tar=0,L_tar=0;
	
	if(fabs(R_tar) >= 180)
	{
		R_tar -= sgn(R_tar) * 360.f;
	}
	if(fabs(L_tar) >= 180)
	{
		L_tar -= sgn(L_tar) * 360.f;
	}
	
	if(rescue_info->last_state == R_IDIE && rescue_info->state == R_IDIE)
	{
		rescue_info->first_in_flag = 0;
		rescue_info->must_restrict = false;
		
		rescue_info->is_rescue = 0;
		
		rescue_info->yaw_save_cnt = 0;
		rescue_info->leg_off_cnt = 0;
	  rescue_info->recline_cnt = 0;
		rescue_info->restrict_cnt = 0;
		rescue_info->stumble_cnt = 0;
		
		rescue_info->stumble_proc = 0;
		rescue_info->recline_proc = 0;
		
		My_Chassis->target->leg_length_r = MAX_LEG_LENGTH;
		My_Chassis->target->leg_length_l = MAX_LEG_LENGTH;
		
		My_Chassis->chassis_PID->length_cal[R_Leg]->kp=My_Chassis->pid_init_parament[R_Leg]->l0_length_kp;
    My_Chassis->chassis_PID->length_cal[L_Leg]->kp=My_Chassis->pid_init_parament[L_Leg]->l0_length_kp;
	  My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp=My_Chassis->pid_init_parament[R_Leg]->l0_length_speed_kp;
	  My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp=My_Chassis->pid_init_parament[L_Leg]->l0_length_speed_kp;
	  My_Chassis->chassis_PID->length_cal[R_Leg]->out_max=My_Chassis->pid_init_parament[R_Leg]->l0_length_outmax;
	  My_Chassis->chassis_PID->length_cal[L_Leg]->out_max=My_Chassis->pid_init_parament[L_Leg]->l0_length_outmax;
	  My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max=My_Chassis->pid_init_parament[R_Leg]->l0_length_speed_outmax;
	  My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max=My_Chassis->pid_init_parament[L_Leg]->l0_length_speed_outmax;
		
		Balance.Flag->Gimbal_Ctrl_Flag = false;
		
		rescue_info->is_rescue = 0;
		
		R_tar = My_R_Link->info->angle->vir_phi0_;
    L_tar = My_L_Link->info->angle->vir_phi0_;
		
	}
	else if(rescue_info->state != rescue_info->last_state)
	{
		rescue_info->first_in_flag = 0;
	}

	
	if(My_Chassis->Posture->info->pitch <= angle2rad(-60) || (My_Chassis->Posture->info->pitch > angle2rad(-60) && My_Chassis->Posture->info->pitch < angle2rad(-0)
		   && fabs(My_Chassis->Posture->info->roll) >= angle2rad(20) && fabs(My_Chassis->Posture->info->roll) <= angle2rad(160)) )
	{
		rescue_info->state = R_STUMBLE;
		rescue_info->is_rescue = 1;
	}
	else if(My_Chassis->Posture->info->pitch >= angle2rad(60) || (My_Chassis->Posture->info->pitch > angle2rad(0) && My_Chassis->Posture->info->pitch < angle2rad(60)
		   && fabs(My_Chassis->Posture->info->roll) >= angle2rad(20) && fabs(My_Chassis->Posture->info->roll) <= angle2rad(160)))
	{
		rescue_info->state = R_RECLINE;
		rescue_info->is_rescue = 1;
	}
	else if(My_Chassis->Posture->info->pitch > angle2rad(-60) && My_Chassis->Posture->info->pitch < angle2rad(60) && fabs(My_Chassis->Posture->info->roll) < angle2rad(20)
		      && (My_R_Link->info->angle->vir_phi0_ <= -73 || My_R_Link->info->angle->vir_phi0_ >=45 
	            || My_L_Link->info->angle->vir_phi0_ <= -73 || My_L_Link->info->angle->vir_phi0_ >=45))
	{
		rescue_info->state = R_LEG_OFF;
		Balance.Flag->Gimbal_Ctrl_Flag = true;
		rescue_info->is_rescue = 0;
	}
	else if(My_Chassis->rescue_info->must_restrict == true || (My_Chassis->Posture->info->pitch > angle2rad(-30) && My_Chassis->Posture->info->pitch < angle2rad(30) 
		      && (My_R_Link->info->angle->vir_phi0_ > -73 && My_R_Link->info->angle->vir_phi0_ <-15 
	            && My_L_Link->info->angle->vir_phi0_ > -73 && My_L_Link->info->angle->vir_phi0_ <-15)))
	{
		rescue_info->state = R_LEG_RESTRACT;
		Balance.Flag->Gimbal_Ctrl_Flag = true;
		rescue_info->is_rescue = 0;
	}
	else if(fabs(My_Chassis->Posture->info->pitch) < angle2rad(20) && My_R_Link->info->angle->vir_phi0_ < 45 && My_R_Link->info->angle->vir_phi0_ > -60 
		        && My_L_Link->info->angle->vir_phi0_ < 45 && My_L_Link->info->angle->vir_phi0_ > -60 && rescue_info->state != R_LEG_RESTRACT )
	{
		Balance.Flag->Rescue_Flag = false;
		Balance.Flag->Gimbal_Ctrl_Flag = true;
		rescue_info->is_rescue = 0;		
		Balance.Flag->Rescue_OK = true;
	}
	
	
	if(Balance.Flag->Gimbal_Ctrl_Flag == true)
	{
		  if(fabs(gimbal.misc.yaw_included_angle) <= PI/2)
			{
				rescue_info->yaw_save_tar = gimbal.info.cfg_info.head_to[0];
			}
		  else if(fabs(gimbal.misc.yaw_included_angle) > PI/2)
			{
				rescue_info->yaw_save_tar = gimbal.info.cfg_info.head_to[4];
			}
		
		
		
//		  rescue_info->yaw_save_tar = Y_ZERO_ANGLE;
	    rescue_info->yaw_save_cnt ++;
      if(fabs(half_cycle(gimbal.yaw->rx_info->motor_angle-rescue_info->yaw_save_tar,2*PI)) <= 5/180*PI && rescue_info->yaw_save_cnt < rescue_info->yaw_cnt_max)
	    {
			  rescue_info->is_rescue = 1;	
	    }
		  else if(rescue_info->yaw_save_cnt >= rescue_info->yaw_cnt_max)
		  {
			  rescue_info->yaw_save_cnt = rescue_info->yaw_cnt_max;
			
			  if(fabs(half_cycle(gimbal.yaw->rx_info->motor_angle-rescue_info->yaw_save_tar,2*PI)) <= rescue_info->yaw_save_range)
			  {
				  rescue_info->yaw_save_tar = gimbal.yaw->rx_info->motor_angle;
				
			  }
			  else
			  {
					if(rescue_info->yaw_save_tar == gimbal.info.cfg_info.head_to[0])
					{
						if(fabs(half_cycle(gimbal.yaw->rx_info->motor_angle- (rescue_info->yaw_save_tar + rescue_info->yaw_save_range),2*PI))
					     <= fabs(half_cycle(gimbal.yaw->rx_info->motor_angle- (rescue_info->yaw_save_tar - rescue_info->yaw_save_range),2*PI)))
				    {
					    rescue_info->yaw_save_tar = rescue_info->yaw_save_tar + rescue_info->yaw_save_range;
				    }
				    else{
					    rescue_info->yaw_save_tar = rescue_info->yaw_save_tar - rescue_info->yaw_save_range;
	
				    }
					}
					else if(rescue_info->yaw_save_tar == gimbal.info.cfg_info.head_to[4])
					{
						if(fabs(half_cycle(gimbal.yaw->rx_info->motor_angle- (rescue_info->yaw_save_tar + rescue_info->yaw_save_range),2*PI))
					     <= fabs(half_cycle(gimbal.yaw->rx_info->motor_angle- (rescue_info->yaw_save_tar - rescue_info->yaw_save_range),2*PI)))
				    {
					    rescue_info->yaw_save_tar = rescue_info->yaw_save_tar + rescue_info->yaw_save_range;
				    }
				    else{
					    rescue_info->yaw_save_tar = rescue_info->yaw_save_tar - rescue_info->yaw_save_range;
	
				    }
					}
				  
			  }
			
			  rescue_info->is_rescue = 1;	
		  }
		
//		  My_Chassis->Leg_Unit[R_Leg]->force->Tp_vir_phi0_ = 0;
//	    My_Chassis->Leg_Unit[L_Leg]->force->Tp_vir_phi0_ = 0;
//	  }
	}

	
	if(rescue_info->is_rescue == 1)
	{
		switch (rescue_info->state)
  	{
		  case R_IDIE:
				rescue_info->restrict_cnt = 0;
			  rescue_info->leg_off_cnt = 0;
			  rescue_info->stumble_cnt = 0;
			  rescue_info->recline_cnt = 0;
			
			  rescue_info->first_in_flag = 0;
			
			  rescue_info->recline_proc = 0;
			  rescue_info->stumble_proc = 0;
		
			  break;
		
		
		  case R_LEG_RESTRACT:
				R_tar = -15.f;
			  L_tar = -15.f;
			
			  My_Chassis->target->leg_length_l = TAR_LEG_LENGTH_INITIAL;
	      My_Chassis->target->leg_length_r = TAR_LEG_LENGTH_INITIAL;
			
			  if(fabs(gimbal.misc.yaw_included_angle) <= PI/2)
			  {
				  rescue_info->yaw_save_tar = gimbal.info.cfg_info.head_to[0];
			  }
		    else if(fabs(gimbal.misc.yaw_included_angle) > PI/2)
			  {
				  rescue_info->yaw_save_tar = gimbal.info.cfg_info.head_to[4];
			  }
			
			  rescue_info->restrict_cnt ++;
			
			  if(fabs(half_cycle(My_R_Link->info->angle->vir_phi0_ - R_tar,360.f)) <= 2.f && fabs(half_cycle(My_L_Link->info->angle->vir_phi0_ - L_tar,360.f)) <= 2.f 
					  && fabs(My_R_Link->info->length->l0 - My_Chassis->target->leg_length_r) <= 0.02f && fabs(My_L_Link->info->length->l0 - My_Chassis->target->leg_length_l) <= 0.02f)
				{
					Balance.Flag->Rescue_Flag = false;
					rescue_info->restrict_cnt = 0;
				
		      Balance.Flag->Rescue_OK = true;
					rescue_info->must_restrict = false;

				}
				else if(rescue_info->restrict_cnt >= 500)
				{
					Balance.Flag->Rescue_Flag = false;
					rescue_info->restrict_cnt = 500;
					
					Balance.Flag->Rescue_OK = true;
					rescue_info->must_restrict = false;
				}
			  break;

			
		  case R_LEG_OFF:
			  if(rescue_info->first_in_flag == 0)
				{
					R_tar = My_R_Link->info->angle->vir_phi0_;
          L_tar = My_L_Link->info->angle->vir_phi0_;

					rescue_info->first_in_flag = 1;
					
				}
				
				My_Chassis->target->leg_length_l = MAX_LEG_LENGTH;
	      My_Chassis->target->leg_length_r = MAX_LEG_LENGTH;
				
				if(fabs(My_R_Link->info->length->l0 - My_Chassis->target->leg_length_r) <= 0.03f && fabs(My_L_Link->info->length->l0 - My_Chassis->target->leg_length_l) <= 0.03f )
				{
					
				
				R_tar += 0.2f;
				L_tar += 0.2f;
				
				if(fabs(R_tar) >= 180)
	      {
		      R_tar -= sgn(R_tar) * 360.f;
	      }
	      if(fabs(L_tar) >= 180)
	      {
		      L_tar -= sgn(L_tar) * 360.f;
	      }
					
			
				rescue_info->leg_off_cnt ++;
				
				if(R_tar >= -71.f && R_tar < 44.f)
				{
					R_tar = -71.f;
				}
				if(L_tar >= -71.f && L_tar < 44.f)
				{
					L_tar = -71.f;
				}
				
				if(fabs(-71.f - My_R_Link->info->angle->vir_phi0_) <= 1.5f && fabs(-71.f - My_L_Link->info->angle->vir_phi0_) <= 1.5f)
				{
					rescue_info->state = R_LEG_RESTRACT;
//				  Balance.Flag->Rescue_Flag = false;
					rescue_info->leg_off_cnt = 0;
					
				}
				else if(rescue_info->leg_off_cnt >= 2500)
				{
					rescue_info->state = R_LEG_RESTRACT;
//					Balance.Flag->Rescue_Flag = false;
					rescue_info->must_restrict = true;
					rescue_info->leg_off_cnt = 2500;
				}
				
			}
			  break;
		
		  case R_STUMBLE:
				if(rescue_info->first_in_flag == 0)
				{
					R_tar = My_R_Link->info->angle->vir_phi0_;
          L_tar = My_L_Link->info->angle->vir_phi0_;

					rescue_info->first_in_flag = 1;
					rescue_info->stumble_proc = 0;
					
				}
				
				My_Chassis->target->leg_length_l = MAX_LEG_LENGTH;
	      My_Chassis->target->leg_length_r = MAX_LEG_LENGTH;
			  
				
				if(rescue_info->stumble_proc == 0)
				{
					rescue_info->stumble_cnt ++;
					
					if(My_R_Link->info->angle->vir_phi0_ >= 175.f || My_R_Link->info->angle->vir_phi0_ <= 60.f)
					{
						R_tar -= 0.2f;
					}
					if(fabs(R_tar) > 180.f)
					{
						R_tar -= sgn(R_tar) * 360.f;
					}
					if(R_tar >= 174.f && R_tar <= 175.f)
					{
						R_tar = 175.f;
					}
					
					
					if(My_L_Link->info->angle->vir_phi0_ >= 175.f || My_L_Link->info->angle->vir_phi0_ <= 60.f)
					{
						L_tar -= 0.2f;
					}
					if(fabs(L_tar) > 180.f)
					{
						L_tar -= sgn(L_tar) * 360.f;
					}
					if(L_tar >= 174.f && L_tar <= 175.f)
					{
						L_tar = 175.f;
					}
					
					if(fabs(half_cycle(My_R_Link->info->angle->vir_phi0_ - 175,360.f))<= 2.f && fabs(half_cycle(My_L_Link->info->angle->vir_phi0_ - 175.f,360.f))<= 2.f 
						   && rescue_info->stumble_cnt < 3000) 
					{
						rescue_info->stumble_proc = 1;
						rescue_info->stumble_cnt = 0;
					}
					else if(rescue_info->stumble_cnt >= 3000)
					{
						rescue_info->stumble_proc = 1;
						rescue_info->stumble_cnt = 0;
					}
					
				}
				
				if(rescue_info->stumble_proc == 1)
				{
					rescue_info->stumble_cnt ++;
					
					R_tar -= 0.2f;
					L_tar -= 0.2f;
					
					
					if(fabs(R_tar) > 180.f)
					{
						R_tar -= sgn(R_tar) * 360.f;
					}
					if(fabs(L_tar) > 180.f)
					{
						L_tar -= sgn(L_tar) * 360.f;
					}
					
					
					if(R_tar <= 105.f && R_tar >= 104.f)
					{
						R_tar = 105.f;
					}
					if(L_tar <= 105.f && L_tar >= 104.f)
					{
						L_tar = 105.f;
					}
					
					if(fabs(half_cycle(My_R_Link->info->angle->vir_phi0_ - 105.f,360.f))<= 1.5f && fabs(half_cycle(My_L_Link->info->angle->vir_phi0_ - 105.f,360.f))<= 1.5f 
						   && rescue_info->stumble_cnt < 1500) 
					{
						rescue_info->stumble_proc = 2;
						rescue_info->stumble_cnt = 0;
						
						rescue_info->state = R_LEG_OFF;
						
					}
					else if(rescue_info->stumble_cnt >= 1500)
					{
						rescue_info->stumble_proc = 2;
						rescue_info->stumble_cnt = 1500;
						
						rescue_info->state = R_LEG_OFF;
					}
					
				}
			  
		
			  break;
		
		
		  case R_RECLINE:
				if(rescue_info->first_in_flag == 0)
				{
					R_tar = My_R_Link->info->angle->vir_phi0_;
          L_tar = My_L_Link->info->angle->vir_phi0_;

					rescue_info->first_in_flag = 1;
					rescue_info->recline_proc = 0;
					
				}
				
				My_Chassis->target->leg_length_l = MAX_LEG_LENGTH;
	      My_Chassis->target->leg_length_r = MAX_LEG_LENGTH;
			  
				if(rescue_info->recline_proc == 0)
				{
					rescue_info->recline_cnt ++;
					
					if(My_R_Link->info->angle->vir_phi0_ >= -60.f || My_R_Link->info->angle->vir_phi0_ <= -150.f)
					{
						R_tar += 0.2f;
					}
					if(My_L_Link->info->angle->vir_phi0_ >= -60.f || My_L_Link->info->angle->vir_phi0_ <= -150.f)
					{
						L_tar += 0.2f;
					}
					
					
					if(fabs(R_tar) > 180.f)
					{
						R_tar -= sgn(R_tar) * 360.f;
					}
					if(fabs(L_tar) > 180.f)
					{
						L_tar -= sgn(L_tar) * 360.f;
					}
					
					
					if(R_tar <= -149.f && R_tar >= -150.f)
					{
						R_tar = -150.f;
					}
					if(L_tar <= -149.f && L_tar >= -150.f)
					{
						L_tar = -150.f;
					}
					
					if(fabs(half_cycle(-150.f- My_R_Link->info->angle->vir_phi0_ ,360.f))<= 2.f && fabs(half_cycle(-150.f - My_L_Link->info->angle->vir_phi0_ ,360.f))<= 2.f 
						   && rescue_info->recline_cnt < 3000) 
					{
						rescue_info->recline_proc = 1;
						rescue_info->recline_cnt = 0;
					}
					else if(rescue_info->recline_cnt >= 3000)
					{
						rescue_info->recline_proc = 1;
						rescue_info->recline_cnt = 0;
					}
					
				}
				
				if(rescue_info->recline_proc == 1)
				{
					rescue_info->recline_cnt ++;
					
					R_tar += 0.2f;
					L_tar += 0.2f;
					
					if(R_tar <= -70.f && R_tar >= -71.f)
					{
						R_tar = -71.f;
					}
					if(L_tar <= -70.f && L_tar >= -71.f)
					{
						L_tar = -71.f;
					}
					
					if(fabs(half_cycle(-71.f-My_R_Link->info->angle->vir_phi0_ ,360.f))<= 1.5f && fabs(half_cycle(-71.f-My_L_Link->info->angle->vir_phi0_ ,360.f))<= 1.5f 
						   && rescue_info->recline_cnt < 1500) 
					{
						rescue_info->recline_proc = 2;
						rescue_info->recline_cnt = 0;
						
						rescue_info->state = R_LEG_RESTRACT;
						
					}
					else if(rescue_info->recline_cnt >= 1500)
					{
						rescue_info->recline_proc = 2;
						rescue_info->recline_cnt = 2000;
						
						rescue_info->state = R_LEG_RESTRACT;
					}
					
				}
				
			  break;
		
		  default:
		  break;
			
	  }
		
		
	}
	
	My_Chassis->target->vir_phi0_r = R_tar;
	My_Chassis->target->vir_phi0_l = L_tar;

	Chassis_Leg_vir_phi0_Cal(My_Chassis);
		
	
	rescue_info->last_state = rescue_info->state;
  Balance.Flag->Last_Rescue_Flag = Balance.Flag->Rescue_Flag;
	if(Balance.Flag->Rescue_Flag == false)
	{
		rescue_info->last_state = R_IDIE; 
		rescue_info->state = R_IDIE;
	
		rescue_info->first_in_flag = 0;
		rescue_info->must_restrict = false;
		
		rescue_info->is_rescue = 0;
		
		rescue_info->yaw_save_cnt = 0;
		rescue_info->leg_off_cnt = 0;
	  rescue_info->recline_cnt = 0;
		rescue_info->restrict_cnt = 0;
		rescue_info->stumble_cnt = 0;
		
		rescue_info->stumble_proc = 0;
		rescue_info->recline_proc = 0;
		
		My_Chassis->chassis_PID->vir_phi0_cal[R_Leg]->integral = 0;
		My_Chassis->chassis_PID->vir_phi0_cal[L_Leg]->integral = 0;

	}
	
	/*腿长控制*/
	
	Chassis_Leg_Length_Strength_Cal(My_Chassis);
	
	/*输入目标Fbl、Tp*/
	My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[R_Leg]->force->F;
	My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[L_Leg]->force->F;
	
	My_Chassis->Leg_Unit[R_Leg]->force->Tp_target = My_Chassis->Leg_Unit[R_Leg]->force->Tp_vir_phi0_;
	My_Chassis->Leg_Unit[L_Leg]->force->Tp_target = My_Chassis->Leg_Unit[L_Leg]->force->Tp_vir_phi0_;
	
	
	My_Chassis->Leg_Unit[R_Leg]->Link->tar_data_update(My_Chassis->Leg_Unit[R_Leg]->Link,My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target,My_Chassis->Leg_Unit[R_Leg]->force->Tp_target);
													
	My_Chassis->Leg_Unit[L_Leg]->Link->tar_data_update(My_Chassis->Leg_Unit[L_Leg]->Link,My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target,My_Chassis->Leg_Unit[L_Leg]->force->Tp_target);

	
	/*计算关节实际输出力矩*/
	My_L_Link->torque_cal(My_L_Link);
	My_R_Link->torque_cal(My_R_Link);
	
	My_Chassis->Leg_Unit[R_Leg]->force->Sd_F_Torque=My_R_Link->info->F_Sd_Output_Torque;
	My_Chassis->Leg_Unit[R_Leg]->force->Sd_B_Torque=My_R_Link->info->B_Sd_Output_Torque;
	My_Chassis->Leg_Unit[L_Leg]->force->Sd_F_Torque=My_L_Link->info->F_Sd_Output_Torque;
	My_Chassis->Leg_Unit[L_Leg]->force->Sd_B_Torque=My_L_Link->info->B_Sd_Output_Torque;
	
	My_Chassis->Leg_Unit[R_Leg]->force->Tw_target = 0;
	My_Chassis->Leg_Unit[L_Leg]->force->Tw_target = 0;
	
	
	/*debug用 begin*/
	My_L_Leg->LQR_cal(My_L_Leg);
	My_R_Leg->LQR_cal(My_R_Leg);
	Chassis_Roll_Control(My_Chassis);
	Chassis_Link_Feedforward_Cal(My_Chassis);
	/*debug用 end*/
	
	
	
}

/**
  * @brief  消去因小腿连接定子转动导致车体位移测量值不准确的误差(比如在腿进行竖直伸缩的时候，会出现位移的偏差）
  * @param  Link_Var_t* Link_Var, float thetal
  * @retval None
  */
static void Stator_Correction_Cal(Chassis_t *My_Chassis)
{
	//右腿
	float thetal=My_Chassis->Leg_Unit[R_Leg]->Straight->info->thetal;
	float phi0=My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->phi0;
	float phi3=My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->phi3;
	Link_t* Link_Var=My_Chassis->Leg_Unit[R_Leg]->Link;
	
	//初中知识，看五连杆的图就可以算出来，需要注意方向
	//stator_angle_now为l3杆与竖直方向的夹角
	Link_Var->info->stator_correction->stator_angle_now = phi3 - phi0 - thetal;
	
	if(Link_Var->info->stator_correction->stator_angle_last == 0)
	{
		Link_Var->info->stator_correction->stator_angular_speed = 0;
	}
	else
	{
		Link_Var->info->stator_correction->stator_angular_speed = \
	(Link_Var->info->stator_correction->stator_angle_now - Link_Var->info->stator_correction->stator_angle_last) / TIME_STEP;
	}
	
	Link_Var->info->stator_correction->stator_angle_last = Link_Var->info->stator_correction->stator_angle_now;
	//伸腿时相当于轮子后退（逆时针），此处stator_angle变小，输出负的stator_bias，则s需要加上一个正的值，故Chassis_State_Var_Update里是减去
	Link_Var->info->stator_correction->stator_bias = R_STATOR_ORDER_CORRECT*Link_Var->info->stator_correction->stator_angular_speed * TIME_STEP
														+Link_Var->info->stator_correction->stator_bias;
	
	//左腿
	thetal=My_Chassis->Leg_Unit[L_Leg]->Straight->info->thetal;
	phi0=My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->phi0;
	phi3=My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->phi3;
	Link_Var=My_Chassis->Leg_Unit[L_Leg]->Link;
	
	Link_Var->info->stator_correction->stator_angle_now =phi3 - phi0 - thetal;
	
	if(Link_Var->info->stator_correction->stator_angle_last == 0)
	{
		Link_Var->info->stator_correction->stator_angular_speed = 0;
	}
	else
	{
		Link_Var->info->stator_correction->stator_angular_speed = \
	(Link_Var->info->stator_correction->stator_angle_now - Link_Var->info->stator_correction->stator_angle_last) / TIME_STEP;
	}
	
	Link_Var->info->stator_correction->stator_angle_last = Link_Var->info->stator_correction->stator_angle_now;

	Link_Var->info->stator_correction->stator_bias = L_STATOR_ORDER_CORRECT*Link_Var->info->stator_correction->stator_angular_speed * TIME_STEP
														+Link_Var->info->stator_correction->stator_bias;
	
}
/**
  * @brief  底盘卸力前加阻尼保护机械结构,记得限制最大输出，或者速度大于一定值不处理
  * @param  None
  * @retval None
  */
static void Chassis_Stop_Damping(Chassis_t* My_Chassis)
{
	float torque_test = 0.f;
	
	torque_test = -My_Chassis->Wheel->motor[0]->rx_info->speed * Wheel_Damping_Coefficient;
	
	My_Chassis->Wheel->motor[0]->tx_info->torque = torque_test;
	
	torque_test = -My_Chassis->Wheel->motor[1]->rx_info->speed * Wheel_Damping_Coefficient;
	
	My_Chassis->Wheel->motor[1]->tx_info->torque = torque_test;

	torque_test = -My_Chassis->Sd->motor[0]->rx_info->speed * R_Sd_Damping_Coefficient;
	
	My_Chassis->Sd->motor[0]->tx_info->torque = torque_test;
	
	torque_test = -My_Chassis->Sd->motor[1]->rx_info->speed * R_Sd_Damping_Coefficient;
	
	My_Chassis->Sd->motor[1]->tx_info->torque = torque_test;
	
	torque_test = -My_Chassis->Sd->motor[2]->rx_info->speed * L_Sd_Damping_Coefficient;
	
	My_Chassis->Sd->motor[2]->tx_info->torque = torque_test;
	
	torque_test = -My_Chassis->Sd->motor[3]->rx_info->speed * L_Sd_Damping_Coefficient;
	
	My_Chassis->Sd->motor[3]->tx_info->torque = torque_test;
}

/**
  * @brief  底盘离线数据处理
  * @param  Chassis_t* chassis, 底盘
  * @retval None
  */
static void Chassis_Offline_Process(Chassis_t* My_Chassis)
{
	/*驱动轮测量值，目标值归零*/
	My_Chassis->Wheel->motor[0]->rx_info->motor_angle_last = 0;
	My_Chassis->Wheel->motor[0]->rx_info->motor_angle_sum = 0;
	
	My_Chassis->Wheel->motor[1]->rx_info->motor_angle_last = 0;
	My_Chassis->Wheel->motor[1]->rx_info->motor_angle_sum = 0;
	
	My_Chassis->target->leg_length_l = TAR_LEG_LENGTH_INITIAL;//腿长目标值改为初始值
	My_Chassis->target->leg_length_r = TAR_LEG_LENGTH_INITIAL;//腿长目标值改为初始值
	My_Chassis->target->yaw = My_Chassis->Posture->info->yaw;//偏航角目标值等于测量值
	My_Chassis->chassis_PID->length_cal[R_Leg]->kp=My_Chassis->pid_init_parament[R_Leg]->l0_length_kp;//防止命令执行过程中进入sleep模式导致pid参数不恢复
	My_Chassis->chassis_PID->length_cal[L_Leg]->kp=My_Chassis->pid_init_parament[L_Leg]->l0_length_kp;
	My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp=My_Chassis->pid_init_parament[R_Leg]->l0_length_speed_kp;
	My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp=My_Chassis->pid_init_parament[L_Leg]->l0_length_speed_kp;
	My_Chassis->target->roll = 0;
	
	My_Chassis->target->s = 0;
	
	My_Chassis->target->sd1 = 0;
	
	My_Chassis->target->yaw_v = 0;
	
	XEstimateKF_Clear(&XEstimateKF);
	/*连杆信息清零*/
	My_Chassis->Leg_Unit[R_Leg]->Link->info->stator_correction->stator_bias = 0;
	My_Chassis->Leg_Unit[R_Leg]->Link->info->stator_correction->stator_angle_last = 0;
	My_Chassis->Leg_Unit[L_Leg]->Link->info->stator_correction->stator_bias = 0;
	My_Chassis->Leg_Unit[L_Leg]->Link->info->stator_correction->stator_angle_last = 0;

	/*pid控制器清零*/
	pid_clear(My_Chassis->chassis_PID->roll_cal[R_Leg]);
	pid_clear(My_Chassis->chassis_PID->yaw_speed_cal[R_Leg]);
	pid_clear(My_Chassis->chassis_PID->length_cal[R_Leg]);
	pid_clear(My_Chassis->chassis_PID->sync_cal[R_Leg]);
	pid_clear(My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg]);
	pid_clear(My_Chassis->chassis_PID->vir_phi0_cal[R_Leg]);

	pid_clear(My_Chassis->chassis_PID->roll_cal[L_Leg]);
	pid_clear(My_Chassis->chassis_PID->yaw_speed_cal[L_Leg]);
	pid_clear(My_Chassis->chassis_PID->length_cal[L_Leg]);
	pid_clear(My_Chassis->chassis_PID->sync_cal[L_Leg]);
	pid_clear(My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg]);
	pid_clear(My_Chassis->chassis_PID->vir_phi0_cal[L_Leg]);
	
	My_Chassis->Leg_Unit[L_Leg]->off_ground_cnt = 0;
	My_Chassis->Leg_Unit[R_Leg]->off_ground_cnt = 0;
	
	My_Chassis->Leg_Unit[R_Leg]->force->F_jump = 0.f;
	My_Chassis->Leg_Unit[L_Leg]->force->F_jump = 0.f;
	
	My_Chassis->Leg_Unit[R_Leg]->force->Tp_vir_phi0_ = 0.f;
	My_Chassis->Leg_Unit[L_Leg]->force->Tp_vir_phi0_ = 0.f;
	
	/*命令状态机清零*/
	Balance.Flag->U_G_Turn_Flag = false;
	Balance.Flag->U_C_Turn_Flag = false;
	Balance.Flag->Jumping_Flag = false;
	Balance.Flag->Knee_Strike_Flag = false;
	Balance.Flag->Fly_Flag = false;
	My_Chassis->jump_info->jump_step=J_IDLE;
	My_Chassis->knee_strike_info->step=Knee_IDLE;
	
	My_Chassis->rescue_info->state = R_IDIE;
	My_Chassis->rescue_info->last_state = R_IDIE;
	
}



/**
  * @brief  腿部角度协调计算
  * @param  Chassis_t* My_Chassis
  * @retval 力Tp
  */
static void Chassis_Leg_Sync_Cal(Chassis_t* My_Chassis)
{
	My_Chassis->chassis_PID->sync_cal[R_Leg]->target = 0;
	My_Chassis->chassis_PID->sync_cal[R_Leg]->measure = R_SYNC_ORDER_CORRECT*(My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->vir_phi0
														-My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->vir_phi0);
	
	My_Chassis->chassis_PID->sync_cal[L_Leg]->target = 0;
	My_Chassis->chassis_PID->sync_cal[L_Leg]->measure = L_SYNC_ORDER_CORRECT*(My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->vir_phi0
														-My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->vir_phi0);
	
	pid_err_cal(My_Chassis->chassis_PID->sync_cal[L_Leg]);
	pid_err_cal(My_Chassis->chassis_PID->sync_cal[R_Leg]);
	single_pid_ctrl(My_Chassis->chassis_PID->sync_cal[R_Leg]);
	single_pid_ctrl(My_Chassis->chassis_PID->sync_cal[L_Leg]);
	My_Chassis->Leg_Unit[R_Leg]->force->Tp_sync = My_Chassis->chassis_PID->sync_cal[R_Leg]->out;
	My_Chassis->Leg_Unit[L_Leg]->force->Tp_sync = My_Chassis->chassis_PID->sync_cal[L_Leg]->out;
}   

/**
  * @brief  双腿vir_phi0_PID计算
  * @param  Chassis_t* My_Chassis
  * @retval 力Tp
  */
static void Chassis_Leg_vir_phi0_Cal(Chassis_t* My_Chassis)
{
	
	/*外环*/
	My_Chassis->chassis_PID->vir_phi0_cal[R_Leg]->measure = My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->vir_phi0_;
	My_Chassis->chassis_PID->vir_phi0_cal[R_Leg]->target = My_Chassis->target->vir_phi0_r;
	
	My_Chassis->chassis_PID->vir_phi0_cal[L_Leg]->measure = My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->vir_phi0_;
	My_Chassis->chassis_PID->vir_phi0_cal[L_Leg]->target = My_Chassis->target->vir_phi0_l;
	
	pid_err_cal(My_Chassis->chassis_PID->vir_phi0_cal[L_Leg]);
	pid_err_cal(My_Chassis->chassis_PID->vir_phi0_cal[R_Leg]);
	
	My_Chassis->chassis_PID->vir_phi0_cal[R_Leg]->err = half_cycle(My_Chassis->chassis_PID->vir_phi0_cal[R_Leg]->err,360.f);
	My_Chassis->chassis_PID->vir_phi0_cal[L_Leg]->err = half_cycle(My_Chassis->chassis_PID->vir_phi0_cal[L_Leg]->err,360.f);
	
	single_pid_ctrl(My_Chassis->chassis_PID->vir_phi0_cal[R_Leg]);
	single_pid_ctrl(My_Chassis->chassis_PID->vir_phi0_cal[L_Leg]);
	
	/*内环*/
	
	My_Chassis->chassis_PID->vir_phi0_speed_cal[R_Leg]->target=My_Chassis->chassis_PID->vir_phi0_cal[R_Leg]->out;
	My_Chassis->chassis_PID->vir_phi0_speed_cal[L_Leg]->target=My_Chassis->chassis_PID->vir_phi0_cal[L_Leg]->out;
	
	My_Chassis->chassis_PID->vir_phi0_speed_cal[R_Leg]->measure=My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->vir_phi0_d1 * Rad2Angle;
	My_Chassis->chassis_PID->vir_phi0_speed_cal[L_Leg]->measure=My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->vir_phi0_d1 * Rad2Angle;
	
	pid_err_cal(My_Chassis->chassis_PID->vir_phi0_speed_cal[L_Leg]);
	pid_err_cal(My_Chassis->chassis_PID->vir_phi0_speed_cal[R_Leg]);
	single_pid_ctrl(My_Chassis->chassis_PID->vir_phi0_speed_cal[R_Leg]);
	single_pid_ctrl(My_Chassis->chassis_PID->vir_phi0_speed_cal[L_Leg]);
	
	My_Chassis->Leg_Unit[R_Leg]->force->Tp_vir_phi0_ = My_Chassis->chassis_PID->vir_phi0_speed_cal[R_Leg]->out;
	My_Chassis->Leg_Unit[L_Leg]->force->Tp_vir_phi0_ = My_Chassis->chassis_PID->vir_phi0_speed_cal[L_Leg]->out;
	
}  
/**
  * @brief  双腿phi0d1PID计算
  * @param  Chassis_t* My_Chassis
  * @retval 力Tp
  */
static void Chassis_Leg_phi0d1_Cal(Chassis_t* My_Chassis)
{
	My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg]->measure = My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->vir_phi0_d1;
	My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg]->target = My_Chassis->target->vir_phi0d1_r;
	
	My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg]->measure = My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->vir_phi0_d1;
	My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg]->target = My_Chassis->target->vir_phi0d1_l;
	
	pid_err_cal(My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg]);
	pid_err_cal(My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg]);
	single_pid_ctrl(My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg]);
	single_pid_ctrl(My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg]);
}

/**
  * @brief  驱动轮转向力矩PID计算
  * @param  Chassis_t* My_Chassis
  * @retval 力Tp
  */
static void Chassis_Wheel_Turn_Cal(Chassis_t* My_Chassis)
{
	
	if(Balance.Flag->chassis_reset == true || (Balance.Flag->Imu_Flag == true && Balance.Flag->Turn_Flag == false && Balance.Flag->S_Turn_Flag == false && Balance.Flag->U_G_Turn_Flag == false && Balance.Flag->U_C_Turn_Flag == false))
	{
		My_Chassis->chassis_PID->yaw_cal[R_Leg]->measure = -gimbal.yaw->rx_info->motor_angle;
		My_Chassis->chassis_PID->yaw_cal[R_Leg]->target = -gimbal.cmd.yaw_mec_tar;
		
		My_Chassis->chassis_PID->yaw_cal[L_Leg]->measure = -gimbal.yaw->rx_info->motor_angle;
		My_Chassis->chassis_PID->yaw_cal[L_Leg]->target = -gimbal.cmd.yaw_mec_tar;
		
		My_Chassis->target->yaw = My_Chassis->Posture->info->yaw;

	}	
	
	else
	{
		My_Chassis->chassis_PID->yaw_cal[R_Leg]->measure = My_Chassis->Posture->info->yaw;
	  My_Chassis->chassis_PID->yaw_cal[R_Leg]->target = My_Chassis->target->yaw;
	
	  My_Chassis->chassis_PID->yaw_cal[L_Leg]->measure = My_Chassis->Posture->info->yaw;
	  My_Chassis->chassis_PID->yaw_cal[L_Leg]->target = My_Chassis->target->yaw;

	}
	
	
	pid_err_cal(My_Chassis->chassis_PID->yaw_cal[R_Leg]);
  pid_err_cal(My_Chassis->chassis_PID->yaw_cal[L_Leg]);
	
	My_Chassis->chassis_PID->yaw_cal[R_Leg]->err=half_cycle(My_Chassis->chassis_PID->yaw_cal[R_Leg]->err,2*PI);
	My_Chassis->chassis_PID->yaw_cal[L_Leg]->err=half_cycle(My_Chassis->chassis_PID->yaw_cal[L_Leg]->err,2*PI);
	
	single_pid_ctrl(My_Chassis->chassis_PID->yaw_cal[R_Leg]);
	single_pid_ctrl(My_Chassis->chassis_PID->yaw_cal[L_Leg]);
	
	My_Chassis->chassis_PID->yaw_speed_cal[R_Leg]->target=My_Chassis->chassis_PID->yaw_cal[R_Leg]->out;
	My_Chassis->chassis_PID->yaw_speed_cal[L_Leg]->target=My_Chassis->chassis_PID->yaw_cal[L_Leg]->out;
	
	My_Chassis->chassis_PID->yaw_speed_cal[R_Leg]->measure=My_Chassis->Posture->info->yaw_v;
	My_Chassis->chassis_PID->yaw_speed_cal[L_Leg]->measure=My_Chassis->Posture->info->yaw_v;
	
	pid_err_cal(My_Chassis->chassis_PID->yaw_speed_cal[R_Leg]);
	pid_err_cal(My_Chassis->chassis_PID->yaw_speed_cal[L_Leg]);
	single_pid_ctrl(My_Chassis->chassis_PID->yaw_speed_cal[R_Leg]);
	single_pid_ctrl(My_Chassis->chassis_PID->yaw_speed_cal[L_Leg]);
	
	My_Chassis->Leg_Unit[R_Leg]->force->Tw_turn= R_TURN_ORDER_CORRECT* My_Chassis->chassis_PID->yaw_speed_cal[R_Leg]->out;
	My_Chassis->Leg_Unit[L_Leg]->force->Tw_turn= L_TURN_ORDER_CORRECT* My_Chassis->chassis_PID->yaw_speed_cal[L_Leg]->out;
	
}   
/**
  * @brief  离地检测，在Chassis_Target_Update处理离地
  * @param  Chassis_t* My_Chassis, 底盘
  * @retval None
  */
static void Chassis_Takeoff_Detect(Chassis_t* My_Chassis)
{
	Link_info_t* R_Link_info = My_Chassis->Leg_Unit[R_Leg]->Link->info;
	Link_info_t* L_Link_info = My_Chassis->Leg_Unit[L_Leg]->Link->info;
	State_info_t* R_Straight_info = My_Chassis->Leg_Unit[R_Leg]->Straight->info;
	State_info_t* L_Straight_info = My_Chassis->Leg_Unit[L_Leg]->Straight->info;
	
	
	/*计算左右驱动轮的支持力*/
	//VMC力+腿重加速度力
	My_Chassis->Leg_Unit[L_Leg]->force->F_support = L_Link_info->force->F_bl_mea * cos(L_Straight_info->thetal) + m_l*(My_Chassis->Posture->info->z_world \
		- (1.f - L_Link_info->centroid->centriod_coefficient) * L_Link_info->length->l0_dot2 * cos(L_Straight_info->thetal));
	My_Chassis->Leg_Unit[R_Leg]->force->F_support = R_Link_info->force->F_bl_mea * cos(R_Straight_info->thetal) + m_l*(My_Chassis->Posture->info->z_world \
		- (1.f - R_Link_info->centroid->centriod_coefficient) * R_Link_info->length->l0_dot2 * cos(R_Straight_info->thetal));
//	L_Link_info->force->F_support = L_Link_info->force->F_bl_mea*arm_cos_f32(Leg_info->thetal_l) + \
//                                  L_Link_info->force->Tp_mea*arm_sin_f32(Leg_info->thetal_l)/L_Link_info->length->l0 + \
//	                                mw * (My_Chassis->Posture->info->z_world - L_Link_info->length->l0_dot2*arm_cos_f32(Leg_info->thetal_l) +
//	                                2*L_Link_info->length->l0_dot*Leg_info->thetald1_l*arm_sin_f32(Leg_info->thetal_l) + \
//	                                L_Link_info->length->l0*Leg_info->thetald2_l*arm_sin_f32(Leg_info->thetal_l) + \
//	                                L_Link_info->length->l0*powf(Leg_info->thetald1_l, 2)*arm_cos_f32(Leg_info->thetal_l));
//	
//	R_Link_info->force->F_support = R_Link_info->force->F_bl_mea*arm_cos_f32(Leg_info->thetal_l) + \
//                                  R_Link_info->force->Tp_mea*arm_sin_f32(Leg_info->thetal_l)/R_Link_info->length->l0 + \
//	                                mw * (My_Chassis->Posture->info->z_world - R_Link_info->length->l0_dot2*arm_cos_f32(Leg_info->thetal_l) +
//	                                2*R_Link_info->length->l0_dot*Leg_info->thetald1_l*arm_sin_f32(Leg_info->thetal_l) + \
//	                                R_Link_info->length->l0*Leg_info->thetald2_l*arm_sin_f32(Leg_info->thetal_l) + \
//	                                R_Link_info->length->l0*powf(Leg_info->thetald1_l, 2)*arm_cos_f32(Leg_info->thetal_l));
	
	//右腿
	if(My_Chassis->Leg_Unit[R_Leg]->force->F_support <= OFF_GROUND_SUPPORT)
	{
		My_Chassis->Leg_Unit[R_Leg]->off_ground_cnt ++;
	}
	else
	{
		My_Chassis->Leg_Unit[R_Leg]->off_ground_cnt = 0;
	}
	if(My_Chassis->Leg_Unit[R_Leg]->off_ground_cnt >= 3)
	{
		My_Chassis->Leg_Unit[R_Leg]->off_ground = true;
		
		My_Chassis->Leg_Unit[R_Leg]->off_ground_cnt = 3;
	}
	else
	{
		My_Chassis->Leg_Unit[R_Leg]->off_ground = false;
	}
	
	//左腿
	if(My_Chassis->Leg_Unit[L_Leg]->force->F_support <= OFF_GROUND_SUPPORT)
	{
		My_Chassis->Leg_Unit[L_Leg]->off_ground_cnt ++;
	}
	else
	{
		My_Chassis->Leg_Unit[L_Leg]->off_ground_cnt = 0;
	}
	if(My_Chassis->Leg_Unit[L_Leg]->off_ground_cnt >= 3)
	{
		My_Chassis->Leg_Unit[L_Leg]->off_ground = true;
		
		My_Chassis->Leg_Unit[L_Leg]->off_ground_cnt = 3;
	}
	else
	{
		My_Chassis->Leg_Unit[L_Leg]->off_ground = false;
	}
}

/**
  * @brief  腿长控制， 计算不同腿长下所需沿杆方向力F
  * @param  Chassis_t* My_Chassis
  */
static void Chassis_Leg_Length_Strength_Cal(Chassis_t* My_Chassis)
{
	Link_t* R_Link_Var = My_Chassis->Leg_Unit[R_Leg]->Link;
	Link_t* L_Link_Var = My_Chassis->Leg_Unit[L_Leg]->Link;
	
	/*位置环*/
	My_Chassis->chassis_PID->length_cal[R_Leg]->measure = R_Link_Var->info->length->l0;
	My_Chassis->chassis_PID->length_cal[L_Leg]->measure = L_Link_Var->info->length->l0;

	/*赋予处理后的目标值 begin*/
	My_Chassis->chassis_PID->length_cal[R_Leg]->target = My_Chassis->target->leg_length_r;
	My_Chassis->chassis_PID->length_cal[L_Leg]->target = My_Chassis->target->leg_length_l;
	
	pid_err_cal(My_Chassis->chassis_PID->length_cal[R_Leg]);
	pid_err_cal(My_Chassis->chassis_PID->length_cal[L_Leg]);
	/*积分分离*/
	if(abs(My_Chassis->chassis_PID->length_cal[R_Leg]->err) > 0.05f)
	{
		My_Chassis->chassis_PID->length_cal[R_Leg]->integral_max = 0;
	}
	else
	{
		My_Chassis->chassis_PID->length_cal[R_Leg]->integral_max = 20;
	}
	single_pid_ctrl(My_Chassis->chassis_PID->length_cal[R_Leg]);
//	
//	/*积分分离*/
	if(abs(My_Chassis->chassis_PID->length_cal[L_Leg]->err) > 0.05f)
	{
		My_Chassis->chassis_PID->length_cal[L_Leg]->integral_max = 0;
	}
	else
	{
		My_Chassis->chassis_PID->length_cal[L_Leg]->integral_max = 20;
	}
	single_pid_ctrl(My_Chassis->chassis_PID->length_cal[L_Leg]);
	
	
	My_Chassis->chassis_PID->length_speed_cal[R_Leg]->measure = R_Link_Var->info->length->l0_dot;//R_Link_Var->info->length->l0;
	My_Chassis->chassis_PID->length_speed_cal[L_Leg]->measure = L_Link_Var->info->length->l0_dot;//L_Link_Var->info->length->l0;
	
	My_Chassis->chassis_PID->length_speed_cal[R_Leg]->target = My_Chassis->chassis_PID->length_cal[R_Leg]->out;//R_Link_Var->info->length->l0;
	My_Chassis->chassis_PID->length_speed_cal[L_Leg]->target = My_Chassis->chassis_PID->length_cal[L_Leg]->out;//L_Link_Var->info->length->l0;

	pid_err_cal(My_Chassis->chassis_PID->length_speed_cal[R_Leg]);
	pid_err_cal(My_Chassis->chassis_PID->length_speed_cal[L_Leg]);
	
	single_pid_ctrl(My_Chassis->chassis_PID->length_speed_cal[R_Leg]);
	single_pid_ctrl(My_Chassis->chassis_PID->length_speed_cal[L_Leg]);
	
	My_Chassis->Leg_Unit[R_Leg]->force->F = My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out;
	My_Chassis->Leg_Unit[L_Leg]->force->F = My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out;
	
}

/**
  * @brief  在所有数据更新后，计算关节、驱动轮所需力矩
  * @param  None
  * @retval None
  */
static void Chassis_Torque_Cal(Chassis_t *My_Chassis)
{
	Link_t* My_L_Link = My_Chassis->Leg_Unit[L_Leg]->Link;
	Link_t* My_R_Link = My_Chassis->Leg_Unit[R_Leg]->Link;
	Straight_Leg_t* R_Straight = My_Chassis->Leg_Unit[R_Leg]->Straight;
	Straight_Leg_t* L_Straight = My_Chassis->Leg_Unit[L_Leg]->Straight;
	/*直腿模型计算，得到驱动轮输出力矩和虚拟关节力矩*/

	/*-----------求Tp_target begin--------*/
	
	Chassis_Leg_Sync_Cal(My_Chassis);
	
	R_Straight->LQR_cal(R_Straight);
	L_Straight->LQR_cal(L_Straight);
	My_Chassis->Leg_Unit[R_Leg]->force->Tp_LQR=R_Straight->get_Tp(R_Straight);
	My_Chassis->Leg_Unit[L_Leg]->force->Tp_LQR=L_Straight->get_Tp(L_Straight);
														
	if(My_Chassis->Leg_Unit[R_Leg]->off_ground == true)              
  {
		My_Chassis->Leg_Unit[R_Leg]->force->Tp_target=R_TP_LQR_ORDER_CORRECT* My_Chassis->Leg_Unit[R_Leg]->force->Tp_LQR;
	}		
	else
	{
		My_Chassis->Leg_Unit[R_Leg]->force->Tp_target= R_TP_LQR_ORDER_CORRECT* My_Chassis->Leg_Unit[R_Leg]->force->Tp_LQR 
													+My_Chassis->Leg_Unit[R_Leg]->force->Tp_sync;
		
	}
	
	if(My_Chassis->Leg_Unit[L_Leg]->off_ground == true)
	{
		My_Chassis->Leg_Unit[L_Leg]->force->Tp_target=L_TP_LQR_ORDER_CORRECT* My_Chassis->Leg_Unit[L_Leg]->force->Tp_LQR;
	}
	else
	{
		My_Chassis->Leg_Unit[L_Leg]->force->Tp_target= L_TP_LQR_ORDER_CORRECT* My_Chassis->Leg_Unit[L_Leg]->force->Tp_LQR 
													+My_Chassis->Leg_Unit[L_Leg]->force->Tp_sync;
	}
		
	
	/*-----------求Tp_target end-----------*/
	
	/*-----------求Fb1_target begin--------*/
	/*腿长控制力计算*/
	Chassis_Leg_Length_Strength_Cal(My_Chassis);
	
	/*roll控制力计算*/
	Chassis_Roll_Control(My_Chassis);
	
	/*前馈计算*/
	Chassis_Link_Feedforward_Cal(My_Chassis);
	
	/*汇总得到Fbl_target*/
	Chassis_Leg_Fbl_Cal(My_Chassis);//含跳跃和离地处理
	
	/*-----------求Fb1_target end--------*/
	
	
	
	/*-----------求Tw_target begin--------*/
	/*驱动轮转向环Tw_turn*/
	Chassis_Wheel_Turn_Cal(My_Chassis);
	My_Chassis->Leg_Unit[R_Leg]->force->Tw_LQR=R_Straight->get_Tw(R_Straight);
	My_Chassis->Leg_Unit[L_Leg]->force->Tw_LQR=L_Straight->get_Tw(L_Straight);
	/* 驱动轮电机最终输出 */
//	if(fabs(My_Chassis->Leg_Unit[R_Leg]->Straight->info->thetal) >= PI * 1/12)
//	{
//		My_Chassis->Leg_Unit[R_Leg]->force->Tw_target=My_Chassis->Leg_Unit[R_Leg]->force->Tw_LQR;
//	}
//  if(Balance.Flag->Rescue_Flag == true)
//	{
//		My_Chassis->Leg_Unit[R_Leg]->force->Tw_target=0;
//	}
	if(My_Chassis->Leg_Unit[R_Leg]->off_ground == true )//离地处理
	{
		My_Chassis->Leg_Unit[R_Leg]->force->Tw_target=0;
	}
	else
	{
		My_Chassis->Leg_Unit[R_Leg]->force->Tw_target=My_Chassis->Leg_Unit[R_Leg]->force->Tw_LQR+My_Chassis->Leg_Unit[R_Leg]->force->Tw_turn;
	}
	
//	if(fabs(My_Chassis->Leg_Unit[L_Leg]->Straight->info->thetal) >= PI * 1/12)
//	{
//		My_Chassis->Leg_Unit[L_Leg]->force->Tw_target=My_Chassis->Leg_Unit[L_Leg]->force->Tw_LQR;
//	}
//	if(Balance.Flag->Rescue_Flag == true)
//	{
//		My_Chassis->Leg_Unit[L_Leg]->force->Tw_target=0;
//	}
	if(My_Chassis->Leg_Unit[L_Leg]->off_ground == true )//离地处理
	{
		My_Chassis->Leg_Unit[L_Leg]->force->Tw_target=0;
	}
	else
	{
		My_Chassis->Leg_Unit[L_Leg]->force->Tw_target=My_Chassis->Leg_Unit[L_Leg]->force->Tw_LQR+My_Chassis->Leg_Unit[L_Leg]->force->Tw_turn;
	}
	/*-----------求Tw_target end--------*/
	
	
	/*建模的Tp方向是顺时针，VMC是逆时针，所以从建模--->VMC要加个负号*/
	My_R_Link->tar_data_update(My_R_Link,My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target,My_Chassis->Leg_Unit[R_Leg]->force->Tp_target);
	My_L_Link->tar_data_update(My_L_Link,My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target,My_Chassis->Leg_Unit[L_Leg]->force->Tp_target);
	
	/*转换为关节力矩,输出到Link结构体的F_Sd_Output_Torque，B_Sd_Output_Torque*/
	My_L_Link->torque_cal(My_L_Link);
	My_R_Link->torque_cal(My_R_Link);
	
//	/* 保护关节 */
//	My_Sd_Position_Nonlinear_Fix(My_Chassis);
	
	
	/* 关节电机最终输出 */
	My_Chassis->Leg_Unit[R_Leg]->force->Sd_F_Torque=My_R_Link->info->F_Sd_Output_Torque+My_Chassis->Leg_Unit[R_Leg]->force->Sd_F_Limit_Tor_Fix;
	My_Chassis->Leg_Unit[R_Leg]->force->Sd_B_Torque=My_R_Link->info->B_Sd_Output_Torque+My_Chassis->Leg_Unit[R_Leg]->force->Sd_B_Limit_Tor_Fix;
	My_Chassis->Leg_Unit[L_Leg]->force->Sd_F_Torque=My_L_Link->info->F_Sd_Output_Torque+My_Chassis->Leg_Unit[L_Leg]->force->Sd_F_Limit_Tor_Fix;
	My_Chassis->Leg_Unit[L_Leg]->force->Sd_B_Torque=My_L_Link->info->B_Sd_Output_Torque+My_Chassis->Leg_Unit[L_Leg]->force->Sd_B_Limit_Tor_Fix;
	
}

/**
  * @brief  沿腿方向力前馈补偿计算
  * @param  Link_Var_t* Link_Var
  * @retval None
  */
float k_inertial= 0.5f;
static void Chassis_Link_Feedforward_Cal(Chassis_t* My_Chassis)
{
	Link_t* R_Link_Var = My_Chassis->Leg_Unit[R_Leg]->Link;
	Link_t* L_Link_Var = My_Chassis->Leg_Unit[L_Leg]->Link;
	State_info_t* R_Straight_info = My_Chassis->Leg_Unit[R_Leg]->Straight->info;

	
//	/*重力前馈*/
//	//杠杆原理，质心越靠近轮子，则支持力提供的力臂越小，所以前馈要更大
	My_Chassis->Leg_Unit[R_Leg]->force->F_gravity = (0.5f * mb + R_Link_Var->info->centroid->centriod_coefficient*m_l) * g * cos(R_Link_Var->info->angle->vir_phi0);
	My_Chassis->Leg_Unit[L_Leg]->force->F_gravity = (0.5f * mb + L_Link_Var->info->centroid->centriod_coefficient*m_l) * g * cos(L_Link_Var->info->angle->vir_phi0);
	
	
	/*侧向力前馈*/
    My_Chassis->Leg_Unit[R_Leg]->force->F_inertial = R_F_INERTIAL_ORDER_CORRECT*((0.5f * mb + R_Link_Var->info->centroid->centriod_coefficient*m_l)*(R_Link_Var->info->length->l0 \
	/ (2.f*Rl))*  My_Chassis->Posture->info->yaw_v * R_Straight_info->sd1)*k_inertial;
	My_Chassis->Leg_Unit[L_Leg]->force->F_inertial = L_F_INERTIAL_ORDER_CORRECT*((0.5f * mb + L_Link_Var->info->centroid->centriod_coefficient*m_l)*(L_Link_Var->info->length->l0 \
	/ (2.f*Rl))*My_Chassis->Posture->info->yaw_v * R_Straight_info->sd1)*k_inertial;
}

/**
  * @brief  计算腿部竖直方向力（期望输出）
  * @param  None
  * @retval None
  */
static void Chassis_Leg_Fbl_Cal(Chassis_t* My_Chassis)
{
//	  if(fabs(My_Chassis->Leg_Unit[R_Leg]->Straight->info->thetal) <= PI * 1/4)
//		{
//			My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[R_Leg]->force->F;
//			My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[L_Leg]->force->F;
//		}
		/* 正常运动 */
	  if(Balance.Flag->Jumping_Flag==false&&
	  	(My_Chassis->Leg_Unit[R_Leg]->off_ground == false||My_Chassis->Leg_Unit[L_Leg]->off_ground ==false))
	  {
	    My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target =	  My_Chassis->Leg_Unit[R_Leg]->force->F
		  					  							+ My_Chassis->Leg_Unit[R_Leg]->force->F_roll
		  			  									+ My_Chassis->Leg_Unit[R_Leg]->force->F_gravity
		  					  							+ My_Chassis->Leg_Unit[R_Leg]->force->F_inertial;
	    My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target =	  My_Chassis->Leg_Unit[L_Leg]->force->F
		  			  									+ My_Chassis->Leg_Unit[L_Leg]->force->F_roll
			    											+ My_Chassis->Leg_Unit[L_Leg]->force->F_gravity
			  							 			  	+ My_Chassis->Leg_Unit[L_Leg]->force->F_inertial;
	   }  
		 /*离地时希望伸长腿，所以额外加上前馈*/
	   else if (Balance.Flag->Jumping_Flag==false&&My_Chassis->Leg_Unit[R_Leg]->off_ground == true)
	   {
		   My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[R_Leg]->force->F + My_Chassis->Leg_Unit[R_Leg]->force->F_gravity;
			 
	   }
		 else if(Balance.Flag->Jumping_Flag==false&&My_Chassis->Leg_Unit[L_Leg]->off_ground ==true)
		 {
			 My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[L_Leg]->force->F + My_Chassis->Leg_Unit[L_Leg]->force->F_gravity;
		 }
	   /* 跳跃时只控制腿长力 */
	   else if(Balance.Flag->Jumping_Flag==true)
	   {
			 if(My_Chassis->jump_info->jump_step == J_EXTEND)
			 {
				 My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[R_Leg]->force->F_gravity + My_Chassis->Leg_Unit[R_Leg]->force->F_jump;
		     My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[L_Leg]->force->F_gravity + My_Chassis->Leg_Unit[R_Leg]->force->F_jump;
			 }
			 else if(My_Chassis->jump_info->jump_step != J_EXTEND && My_Chassis->Leg_Unit[R_Leg]->off_ground == true&&My_Chassis->Leg_Unit[L_Leg]->off_ground ==true)
			 {
				 My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[R_Leg]->force->F; 
			   My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[L_Leg]->force->F; 
			 } 
			 else{
			   My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target =	  My_Chassis->Leg_Unit[R_Leg]->force->F
		  					  							+ My_Chassis->Leg_Unit[R_Leg]->force->F_roll
		  			  									+ My_Chassis->Leg_Unit[R_Leg]->force->F_gravity
		  					  							+ My_Chassis->Leg_Unit[R_Leg]->force->F_inertial;
	       My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target =	  My_Chassis->Leg_Unit[L_Leg]->force->F
		  			  									+ My_Chassis->Leg_Unit[L_Leg]->force->F_roll
			    											+ My_Chassis->Leg_Unit[L_Leg]->force->F_gravity
			  							 			  	+ My_Chassis->Leg_Unit[L_Leg]->force->F_inertial;
				 
			 }
		   
	   }
		  
	
	
}

/**
  * @brief  roll控制
  * @param  Link_Var_t* Link_Var
  * @retval 力F
  * @note  右腿减左腿加
  */
static void Chassis_Roll_Control(Chassis_t* My_Chassis)
{
	
	My_Chassis->chassis_PID->roll_cal[R_Leg]->measure = My_Chassis->Posture->info->roll;
	
	My_Chassis->chassis_PID->roll_cal[L_Leg]->measure = My_Chassis->Posture->info->roll;
	
	My_Chassis->chassis_PID->roll_cal[R_Leg]->target = My_Chassis->target->roll;
  
    My_Chassis->chassis_PID->roll_cal[L_Leg]->target = My_Chassis->target->roll;	

	pid_err_cal(My_Chassis->chassis_PID->roll_cal[R_Leg]);
	single_pid_ctrl(My_Chassis->chassis_PID->roll_cal[R_Leg]);
	
	pid_err_cal(My_Chassis->chassis_PID->roll_cal[L_Leg]);
	single_pid_ctrl(My_Chassis->chassis_PID->roll_cal[L_Leg]);
	if(My_Chassis->Leg_Unit[R_Leg]->off_ground!=true)
	{
		My_Chassis->Leg_Unit[R_Leg]->force->F_roll = R_TP_Roll_ORDER_CORRECT*My_Chassis->chassis_PID->roll_cal[R_Leg]->out;
	}
	else
	{
		My_Chassis->Leg_Unit[R_Leg]->force->F_roll = 0;
	}
	
	if(My_Chassis->Leg_Unit[L_Leg]->off_ground!=true)
	{
		My_Chassis->Leg_Unit[L_Leg]->force->F_roll = L_TP_Roll_ORDER_CORRECT*My_Chassis->chassis_PID->roll_cal[L_Leg]->out;
	}
	else
	{
		My_Chassis->Leg_Unit[L_Leg]->force->F_roll = 0;
	}

	
	
	
}

/**
  * @brief  Phi1由前关节电机编码器值转换得到
  * @param  Link_e My_Link_e 杆的左右标志位
  * @param  Motor_Ktech_t* my_motor 前关节电机
  * @retval 转换后的角度phi1
  */
static float My_Phi1_Transform(Leg_e My_Leg_e, Motor_DM_t* my_motor)
{
	float phi1 = 0.f;
	if(My_Leg_e == R_Leg)
	{
		phi1 = R_F_TIME*my_motor->rx_info->motor_angle + R_F_HORIZON_ANGLE_ORDER_CORRECT* R_F_HORIZON_ANGLE;
		
		
		
	}
	else if(My_Leg_e == L_Leg)
	{
		phi1 = L_F_TIME*my_motor->rx_info->motor_angle + L_F_HORIZON_ANGLE_ORDER_CORRECT* L_F_HORIZON_ANGLE;
		
	
	}
	
	if(phi1 > PI)
	{
		phi1 -= 2* PI;
	}
	if(phi1 < -PI)
	{
		phi1 += 2*PI;
	}
	
	if(phi1 >= 0)
	{
		 phi1 = (float)PI - phi1;
	}
  if(phi1 < 0)
	{
		phi1 = (float)(-PI) - phi1;
	}
	
	return phi1;
	
}


/**
  * @brief  Phi4由后关节电机编码器值转换得到
  * @param  Link_e My_Link_e 杆的左右标志位
  * @param  Motor_Ktech_t* my_motor 前关节电机
  * @retval 转换后的角度phi4
  */
static float My_Phi4_Transform(Leg_e My_Leg_e, Motor_DM_t* my_motor)
{
	float phi4 = 0.f;
	if(My_Leg_e == R_Leg)
	{
		phi4 = R_B_TIME*my_motor->rx_info->motor_angle +R_B_HORIZON_ANGLE_ORDER_CORRECT* R_B_HORIZON_ANGLE;
	
		
	}
	else if(My_Leg_e == L_Leg)
	{
		phi4 = L_B_TIME*my_motor->rx_info->motor_angle +L_B_HORIZON_ANGLE_ORDER_CORRECT* L_B_HORIZON_ANGLE;
		
	}
	
	
	
	if(phi4 > PI)
	{
		phi4 -= 2*PI;
	}
	if(phi4 < -PI)
	{
		phi4 += 2*PI;
	}
	
	
	return phi4;
	
}



/**
  * @brief  底盘电机组离线检测
  * @param  Chassis_t* My_Chassis, 底盘
  * @retval None
  */
static void Chassis_Motor_Group_Offline_Check(Chassis_t* My_Chassis)
{
	if(My_Chassis->Sd->motor[0]->state->status == DEV_OFFLINE || My_Chassis->Sd->motor[2]->state->status == DEV_OFFLINE \
		 || My_Chassis->Sd->motor[3]->state->status == DEV_OFFLINE || My_Chassis->Sd->motor[1]->state->status == DEV_OFFLINE)
	{
		My_Chassis->state->sd_state = DEV_OFFLINE;
	}
	else
	{
		My_Chassis->state->sd_state = DEV_ONLINE;
	}
	if(My_Chassis->Wheel->motor[0]->state->status == DEV_OFFLINE || My_Chassis->Wheel->motor[1]->state->status == DEV_OFFLINE)
	{
		My_Chassis->state->wheel_state = DEV_OFFLINE;
	}
	else
	{
		My_Chassis->state->wheel_state = DEV_ONLINE;
	}
}

/**
  * @brief  底盘转向目标值控制，角速度积分成角度
  * @param  Chassis_t* My_Chassis
  * @retval None
  */
float turn_speed = 0.04f;
static void Chassis_Yaw_Target_Process_All(Chassis_t* My_Chassis)
{
	static float turn_limit = 0;
	static bool last_s_flag = false;
	static uint32_t start_time = 0;
	
	if(Balance.Flag->chassis_reset == true)
	{
		My_Chassis->chassis_PID->yaw_cal[R_Leg]->kp = 4.f;
		My_Chassis->chassis_PID->yaw_cal[L_Leg]->kp = 4.f;
		My_Chassis->chassis_PID->yaw_speed_cal[R_Leg]->out_max = 5.f;
		My_Chassis->chassis_PID->yaw_speed_cal[L_Leg]->out_max = 5.f;
	}
	else{
	  My_Chassis->chassis_PID->yaw_cal[R_Leg]->kp = 7.f;
		My_Chassis->chassis_PID->yaw_cal[L_Leg]->kp = 7.f;
		My_Chassis->chassis_PID->yaw_speed_cal[R_Leg]->out_max = 100.f;
		My_Chassis->chassis_PID->yaw_speed_cal[L_Leg]->out_max = 100.f;
	}
	

	switch(My_Chassis->mode)
	{
		case C_Sleep:
		My_Chassis->target->yaw_v = 0;

		break;
	
		case C_Boss:
			if(Balance.Flag->U_G_Turn_Flag == false && Balance.Flag->U_C_Turn_Flag == false)
			{
				if(Balance.ctrl == RC_CTRL)
			  {
				  My_Chassis->target->yaw_v = -((float)My_Chassis->rc_input->ch0_now / 660.f) * MAX_SPIN_SPEED;
			  }
			  else if(Balance.ctrl == KEY_CTRL)
			  {  
				  My_Chassis->target->yaw_v = -rc_sensor.info->mouse_vx * turn_speed;
				  My_Chassis->target->yaw_v = constrain(My_Chassis->target->yaw_v, -MAX_SPIN_SPEED,MAX_SPIN_SPEED);
			  }
				
			}
			else{
			  My_Chassis->target->yaw_v = 0;
			}
			
		break;
		
		case C_Test:
			#ifndef VISION_TEST
		
	    if(Balance.Flag->U_G_Turn_Flag == false && Balance.Flag->U_C_Turn_Flag == false)
			{
				if(Balance.ctrl == RC_CTRL)
			  {
				  My_Chassis->target->yaw_v = -((float)My_Chassis->rc_input->ch0_now / 660.f) * MAX_SPIN_SPEED;
			  }
			  else if(Balance.ctrl == KEY_CTRL)
			  {  
				  My_Chassis->target->yaw_v = -rc_sensor.info->mouse_vx * turn_speed;
				  My_Chassis->target->yaw_v = constrain(My_Chassis->target->yaw_v, -MAX_SPIN_SPEED,MAX_SPIN_SPEED);
			  }
				
			}
			else{
			  My_Chassis->target->yaw_v = 0;
			}
			
		  #endif
		break;
		
		case C_Knee_Strike:
			if(Balance.Flag->U_G_Turn_Flag == false && Balance.Flag->U_C_Turn_Flag == false)
			{
				if(Balance.ctrl == RC_CTRL)
			  {
				  My_Chassis->target->yaw_v = -((float)My_Chassis->rc_input->ch0_now / 660.f) * MAX_SPIN_SPEED;
			  }
			  else if(Balance.ctrl == KEY_CTRL)
			  {  
				  My_Chassis->target->yaw_v = -rc_sensor.info->mouse_vx * turn_speed;
				  My_Chassis->target->yaw_v = constrain(My_Chassis->target->yaw_v, -MAX_SPIN_SPEED,MAX_SPIN_SPEED);
			  }
				
			}
			else{
			  My_Chassis->target->yaw_v = 0;
			}
		
		break;
		
		case C_Follow:
		break;
		
		case C_Turn:
			if(Balance.Flag->Turn_Flag == true)
			{
				My_Chassis->target->yaw_v = 8.f;
			}
			else if(Balance.Flag->S_Turn_Flag == true)
			{
				if(last_s_flag == false && Balance.Flag->S_Turn_Flag == true)
	      {
		      start_time = HAL_GetTick();
	      }
				
			  My_Chassis->target->yaw_v = 8 - 2*arm_cos_f32(PI/1000*(HAL_GetTick() - start_time));
				
			}
			break;
			
		case C_Fly:
      if(Balance.Flag->Mec_Flag == true )
			{
				if(Balance.Flag->U_G_Turn_Flag == false && Balance.Flag->U_C_Turn_Flag == false)
			  {
				  if(Balance.ctrl == RC_CTRL)
			    {
				    My_Chassis->target->yaw_v = -((float)My_Chassis->rc_input->ch0_now / 660.f) * MAX_SPIN_SPEED;
			    }
			    else if(Balance.ctrl == KEY_CTRL)
			    {  
				    My_Chassis->target->yaw_v = -rc_sensor.info->mouse_vx * turn_speed;
				    My_Chassis->target->yaw_v = constrain(My_Chassis->target->yaw_v, -MAX_SPIN_SPEED,MAX_SPIN_SPEED);
			    }
				
			  }
			  else{
			    My_Chassis->target->yaw_v = 0;
			  }
			}		
      break;			
		default:
		break;
	}
	
	if(judge.info->power_heat_data.buffer_energy < 55.f)
	{
		turn_limit = (float)judge.info->power_heat_data.buffer_energy / 60.f;
		
		My_Chassis->target->yaw_v *= turn_limit;
	} 
	

	//My_Chassis->target->yaw目标值输出
	if(fabs(My_Chassis->target->yaw_v) >= MAX_SPIN_SPEED/660.f/10.f)
	{
		My_Chassis->target->yaw += My_Chassis->target->yaw_v*TIME_STEP;
		do{
		  My_Chassis->target->yaw=half_cycle(My_Chassis->target->yaw,2*PI);

		}while(My_Chassis->target->yaw > PI || My_Chassis->target->yaw < -PI);
		
	}
	
	last_s_flag = Balance.Flag->S_Turn_Flag;
}
/**
  * @brief  赋予左右腿腿长目标值
  * @param  Chassis_t* My_Chassis
  * @retval None
  */
static void Chassis_Leg_Length_Target_Process(Chassis_t* My_Chassis)
{
  static bool last_fly = false;
	static bool offland = false;

	
	if(Balance.Flag->Jumping_Flag == false && Balance.Flag->Knee_Strike_Flag == false && Balance.Flag->Rescue_Flag ==false)
	{
		if(My_Chassis->Leg_Unit[R_Leg]->off_ground == true && My_Chassis->Leg_Unit[L_Leg]->off_ground == false && offland == false)
	  {
	  	My_Chassis->target->leg_length_r = MID_LEG_LENGTH;
	  }
	
	  if(My_Chassis->Leg_Unit[R_Leg]->off_ground == false && My_Chassis->Leg_Unit[L_Leg]->off_ground == true && offland == false)
	  {
		  My_Chassis->target->leg_length_l = MID_LEG_LENGTH;
	  }
	
		if(My_Chassis->Leg_Unit[R_Leg]->off_ground == true && My_Chassis->Leg_Unit[L_Leg]->off_ground == true)
		{
			if(My_Chassis->target->leg_length_r >= MID_LEG_LENGTH && My_Chassis->target->leg_length_l >= MID_LEG_LENGTH)
			{
				My_Chassis->target->leg_length_r = MID_LEG_LENGTH;
			  My_Chassis->target->leg_length_l = MID_LEG_LENGTH;
				
			}
			else{
			  My_Chassis->target->leg_length_r += 0.004f;
			  My_Chassis->target->leg_length_l += 0.004f;
				
				if(My_Chassis->target->leg_length_r >= MID_LEG_LENGTH)
				{
					My_Chassis->target->leg_length_r = MID_LEG_LENGTH;
				}
				if(My_Chassis->target->leg_length_l >= MID_LEG_LENGTH)
				{
					My_Chassis->target->leg_length_l = MID_LEG_LENGTH;
				}
				
			}
			
			offland = true;
		}
		else if(My_Chassis->Leg_Unit[R_Leg]->off_ground == false && My_Chassis->Leg_Unit[L_Leg]->off_ground == false && offland == true)
		{
			My_Chassis->target->leg_length_r -= 0.002f;
      My_Chassis->target->leg_length_l -= 0.002f;
			
			if((My_Chassis->Leg_Unit[R_Leg]->Link->info->length->l0 + My_Chassis->Leg_Unit[L_Leg]->Link->info->length->l0)/2 <= TAR_LEG_LENGTH_INITIAL)
			{
				Balance.Flag->Fly_Flag = false;
        My_Chassis->target->leg_length_r = TAR_LEG_LENGTH_INITIAL;
			  My_Chassis->target->leg_length_l = TAR_LEG_LENGTH_INITIAL;
				
				offland = false;
      }
		}
	}
	
	
  if(Balance.Flag->Fly_Flag == true && last_fly == false)
	{
		My_Chassis->target->leg_length_l = MID_LEG_LENGTH;
		My_Chassis->target->leg_length_r = MID_LEG_LENGTH;
	}
	else if(Balance.Flag->Fly_Flag == false && last_fly == true)
	{
		My_Chassis->target->leg_length_r = TAR_LEG_LENGTH_INITIAL;
		My_Chassis->target->leg_length_l = TAR_LEG_LENGTH_INITIAL;
	}
	
	  if(Balance.Flag->Leg_length_ctrl_Flag==true)
	  {
			if(Balance.ctrl == RC_CTRL)
			{
				My_Chassis->target->leg_length_l += ((float)My_Chassis->rc_input->ch2_now / 660.f) * MAX_LIFT_SPEED * TIME_STEP;
		    My_Chassis->target->leg_length_r += ((float)My_Chassis->rc_input->ch2_now / 660.f) * MAX_LIFT_SPEED * TIME_STEP;
			}
		  
	  }


	
	/*限制腿长极限值*/
	if(My_Chassis->target->leg_length_l > MAX_LEG_LENGTH)
	{
		My_Chassis->target->leg_length_l = MAX_LEG_LENGTH;
	}
	else if(My_Chassis->target->leg_length_l < MIN_LEG_LENGTH)
	{

		My_Chassis->target->leg_length_l = MIN_LEG_LENGTH;
	}
	
	if(My_Chassis->target->leg_length_r > MAX_LEG_LENGTH)
	{
		My_Chassis->target->leg_length_r = MAX_LEG_LENGTH;
	}
	else if(My_Chassis->target->leg_length_r < MIN_LEG_LENGTH)
	{

		My_Chassis->target->leg_length_r= MIN_LEG_LENGTH;
	}
	
	last_fly = Balance.Flag->Fly_Flag;

}	

/**
  * @brief  由控制层到电机层，将计算出来的扭矩赋给相应电机
  * @param  None
  * @retval None
  */
static void Chassis_Set_Torque(Chassis_t* My_Chassis)
{
	My_Chassis->Sd->motor[R_F_Sd_M]->tx_info->torque = My_Chassis->Leg_Unit[R_Leg]->force->Sd_F_Torque * R_F_ORDER_CORRECT + My_Chassis->Leg_Unit[R_Leg]->Link->info->force->Spring_T_Feed_Front;
	My_Chassis->Sd->motor[R_B_Sd_M]->tx_info->torque = My_Chassis->Leg_Unit[R_Leg]->force->Sd_B_Torque * R_B_ORDER_CORRECT - My_Chassis->Leg_Unit[R_Leg]->Link->info->force->Spring_T_Feed_Back;
	My_Chassis->Sd->motor[L_F_Sd_M]->tx_info->torque = My_Chassis->Leg_Unit[L_Leg]->force->Sd_F_Torque * L_F_ORDER_CORRECT - My_Chassis->Leg_Unit[L_Leg]->Link->info->force->Spring_T_Feed_Front;
	My_Chassis->Sd->motor[L_B_Sd_M]->tx_info->torque = My_Chassis->Leg_Unit[L_Leg]->force->Sd_B_Torque * L_B_ORDER_CORRECT + My_Chassis->Leg_Unit[L_Leg]->Link->info->force->Spring_T_Feed_Back;

	My_Chassis->Wheel->motor[R_WHEEL_M]->tx_info->torque = My_Chassis->Leg_Unit[R_Leg]->force->Tw_target*R_W_ORDER_CORRECT;
	My_Chassis->Wheel->motor[L_WHEEL_M]->tx_info->torque = My_Chassis->Leg_Unit[L_Leg]->force->Tw_target*L_W_ORDER_CORRECT;
	
//	My_Chassis->Sd->motor[R_F_Sd_M]->tx_info->torque = 0;
//	My_Chassis->Sd->motor[R_B_Sd_M]->tx_info->torque = 0;
//	
//	My_Chassis->Sd->motor[L_F_Sd_M]->tx_info->torque = 0;
//  My_Chassis->Sd->motor[L_B_Sd_M]->tx_info->torque = 0;

//  My_Chassis->Wheel->motor[R_WHEEL_M]->tx_info->torque = 0;
//  My_Chassis->Wheel->motor[L_WHEEL_M]->tx_info->torque = 0;

}

/**
  * @brief  电机层目标力矩赋0，不发送
  */
static void Chassis_Motor_Set_Sleep(Chassis_t* My_Chassis)
{
	My_Chassis->Sd->motor[R_F_Sd_M]->tx_info->torque = 0;
	My_Chassis->Sd->motor[R_B_Sd_M]->tx_info->torque = 0;
	My_Chassis->Sd->motor[L_F_Sd_M]->tx_info->torque = 0;
	My_Chassis->Sd->motor[L_B_Sd_M]->tx_info->torque = 0;
	
	My_Chassis->Wheel->motor[R_WHEEL_M]->tx_info->torque = 0;
	My_Chassis->Wheel->motor[L_WHEEL_M]->tx_info->torque = 0;
}

/**
  * @brief  底盘输入值步进限幅滤波
  * @param  Chassis_t* My_Chassis
  * @retval None
  */
static void Chassis_Rc_Input_Update(Chassis_t* My_Chassis)
{
	Chassis_Rc_Input_t* rc_input = My_Chassis->rc_input;
	
	/*偏航*/
	rc_input->ch0_now = rc_sensor.info->ch0;
//	rc_input->ch0_now=step_limit_filter(rc_input->ch0_now,rc_input->ch0_last,2);
//	rc_input->ch0_last = rc_input->ch0_now;
	
	/*俯仰*/
	rc_input->ch1_now = rc_sensor.info->ch1;
//	rc_input->ch1_now=step_limit_filter(rc_input->ch1_now,rc_input->ch1_last,2);
//	rc_input->ch1_last = rc_input->ch1_now;
	
	/*腿长*/
	rc_input->ch2_now = rc_sensor.info->ch2;
//	rc_input->ch2_now=step_limit_filter(rc_input->ch0_now,rc_input->ch2_last,2);
//	rc_input->ch2_last = rc_input->ch2_now;

	
	/*前后*/
	rc_input->ch3_now = rc_sensor.info->ch3;
//	rc_input->ch3_now=step_limit_filter(rc_input->ch3_now,rc_input->ch3_last,2);
//	rc_input->ch3_last = rc_input->ch3_now;

	
	
}
float W_S_now = 0, W_S_last = 0,D_A_now = 0,D_A_last = 0;
void Key_Change(void)
{
	W_S_now = rc_sensor.info->W.cnt - rc_sensor.info->S.cnt;
	
	if(abs(W_S_now - W_S_last) > 1)
	{
		W_S_now = 2.f * sgn(rc_sensor.info->W.cnt - rc_sensor.info->S.cnt) + W_S_last;
		W_S_last = W_S_now;
	}
	else{
		W_S_last = W_S_now;
	
	}
	
	D_A_now = rc_sensor.info->D.cnt - rc_sensor.info->A.cnt;
	
	if(abs(D_A_now - D_A_last) > 1)
	{
		D_A_now = 2.f * sgn(rc_sensor.info->D.cnt - rc_sensor.info->A.cnt) + D_A_last;
		D_A_last = D_A_now;
	}
	else{
		D_A_last = D_A_now;
	
	}

}




/**
  * @brief  控速度
  * @param  Chassis_t* My_Chassis, 底盘
  * @retval None
  */
uint8_t start_power = 0,end_power = 0;
static void Chassis_sd1_Target_Update(Chassis_t* My_Chassis)
{
	static float head_to = 1.f;
	
	if(fabs(gimbal.misc.yaw_included_angle) <= PI/2)
	{
		head_to = 1.f;
	}
	else if(fabs(gimbal.misc.yaw_included_angle) > PI/2)
	{
		head_to = -1.f;
	}
	
	if(Balance.Flag->Fly_Flag == true || Balance.Flag->Reserve_Fly_Flag == true)
	{
		My_Chassis->target->velocity_max = 2.5f;
	}
	else{
	  My_Chassis->target->velocity_max = 2.4f;
	}

	
	if(end_power == 0)
	{
		if(Balance.Flag->Turn_Flag == true || Balance.Flag->S_Turn_Flag == true)
	  {
		  My_Chassis->target->sd1 = Chassis_S_Turn_sdl_update(My_Chassis);
	  }
	  else
	  {
		  if(Balance.ctrl == RC_CTRL)
	    {
		    My_Chassis->target->sd1 = head_to * RC_INPUT_SD1_ORDER_CORRECT * (float)My_Chassis->rc_input->ch3_now / 660.f * My_Chassis->target->velocity_limit;
	    }
	    else if(Balance.ctrl == KEY_CTRL)
	    {
	      My_Chassis->target->sd1 = head_to * KEY_INPUT_SD1_ORDER_CORRECT * 1.f *((float)My_Chassis->rc_input->w_now - (float)My_Chassis->rc_input->s_now) /(float)rc_sensor.info->S.cnt_max * My_Chassis->target->velocity_limit;
		
	    }
		
		  My_Chassis->target->sd1 = constrain(My_Chassis->target->sd1,-My_Chassis->target->velocity_limit , My_Chassis->target->velocity_limit);	
	  }
	}
	else if(end_power == 1)
	{
		if(Balance.Flag->Turn_Flag == true || Balance.Flag->S_Turn_Flag == true)
	  {
		  My_Chassis->target->sd1 = Chassis_S_Turn_sdl_update(My_Chassis);
	  }
	  else
	  {
		  if(Balance.ctrl == RC_CTRL)
	    {
		    My_Chassis->target->sd1 = head_to * RC_INPUT_SD1_ORDER_CORRECT * (float)My_Chassis->rc_input->ch3_now / 660.f * My_Chassis->target->velocity_max;
	    }
	    else if(Balance.ctrl == KEY_CTRL)
	    {
	      My_Chassis->target->sd1 = head_to * KEY_INPUT_SD1_ORDER_CORRECT * 1.f *((float)My_Chassis->rc_input->w_now - (float)My_Chassis->rc_input->s_now) /(float)rc_sensor.info->S.cnt_max * My_Chassis->target->velocity_max;
		
	    }
		
		  My_Chassis->target->sd1 = constrain(My_Chassis->target->sd1,-My_Chassis->target->velocity_max , My_Chassis->target->velocity_max);	
	  }
	}
	
	if(judge.info->power_heat_data.buffer_energy <= 55 && judge.info->power_heat_data.buffer_energy >= 0)//范围
	{
		if(start_power == 0)//第一次进入
		{
			start_power = 1;
       My_Chassis->target->velocity_limit = My_Chassis->target->velocity_max;
			Balance.Flag->Power_Limit_Flag = true;
//				My_Chassis.target->velocity_limit = abs(My_Chassis.target->velocity);
		}
		end_power = 0;
			
	}
		
	else
	{
		start_power = 0;
		Balance.Flag->Power_Limit_Flag = false;
		end_power = 1;
	}
	
	Chassis_Power_Limit(My_Chassis);
	

	if((float)fabs(My_Chassis->target->sd1 / MAX_STRAIGHT_SPEED) >= 0.2f)
	{
		My_Chassis->target->s = My_Chassis->Leg_Unit[R_Leg]->Straight->info->s;
	}
	

}


/**
  * @brief  底盘初始化控制处理
  * @param  Chassis_t* My_Chassis, 底盘
  * @retval None
  */
static void Chassis_Init_Ctrl(Chassis_t* My_Chassis)
{
	My_Chassis->Leg_Unit[R_Leg]->off_ground = false;
	My_Chassis->Leg_Unit[L_Leg]->off_ground = false;
	
//	if(My_Chassis->mode == C_Init)
//	{
//		if(My_Chassis->init_state == Front)
//		{
//			My_Chassis->target->leg_length = MIN_LEG_LENGTH;
//		}
//		else if(My_Chassis->init_state == Behind)
//		{
//			My_Chassis->target->leg_length = MIN_LEG_LENGTH;
//		}
//		else if(My_Chassis->init_state == L_Front)
//		{
//			My_Chassis->target->leg_length = MIN_LEG_LENGTH;
//		}
//		else if(My_Chassis->init_state == R_Front)
//		{
//			My_Chassis->target->leg_length = MIN_LEG_LENGTH;
//		}
//	}
	My_Chassis->target->yaw_v = 0;
	My_Chassis->target->sd1 = 0;
	My_Chassis->target->roll = 0;
	My_Chassis->target->yaw = My_Chassis->Posture->info->yaw;//偏航角目标值等于测量值
	

	float thetal_err = My_Chassis->Leg_Unit[L_Leg]->Straight->X_info->X_err_mat_storage[X_thetal];
	float thetar_err = My_Chassis->Leg_Unit[R_Leg]->Straight->X_info->X_err_mat_storage[X_thetal];
	static uint8_t cnt = 0;
	

	if(My_Chassis->reset_struct->reset_cnt == 0)
	{
		if(My_Chassis->Leg_Unit[L_Leg]->Straight->info->thetal <= 0.f && My_Chassis->Leg_Unit[R_Leg]->Straight->info->thetal <= 0.f)
		{
			My_Chassis->init_state = Front;
		}
		else if(My_Chassis->Leg_Unit[L_Leg]->Straight->info->thetal > 0.f && My_Chassis->Leg_Unit[R_Leg]->Straight->info->thetal > 0.f)
		{
			My_Chassis->init_state = Behind;
		}
		else if(My_Chassis->Leg_Unit[L_Leg]->Straight->info->thetal < 0.f && My_Chassis->Leg_Unit[R_Leg]->Straight->info->thetal > 0.f)
		{
			My_Chassis->init_state = L_Front;
		}
		else if(My_Chassis->Leg_Unit[L_Leg]->Straight->info->thetal > 0.f && My_Chassis->Leg_Unit[R_Leg]->Straight->info->thetal < 0.f)
		{
			My_Chassis->init_state = R_Front;
		}
	}
	My_Chassis->reset_struct->reset_cnt ++;
	if((abs(thetal_err) < 0.75f && abs(thetar_err) < 0.75f) ||My_Chassis->reset_struct->reset_cnt > 800)
	{
		cnt ++;
		if(cnt > 20 || My_Chassis->reset_struct->reset_cnt > 800)
		{
			My_Chassis->target->leg_length_r = TAR_LEG_LENGTH_INITIAL;
			My_Chassis->target->leg_length_l = TAR_LEG_LENGTH_INITIAL;
			My_Chassis->reset_struct->reset_state = true;
		}
	}
	else
	{
		cnt = 0;
	}
}

static int binarySearchClosestLess(float arr[], int size, float target, float mean);
/**
  * @brief  目标速度限制以限制向心力
  * @param  Chassis_t* My_Chassis, 底盘
  * @retval None
  */
static void Chassis_Speed_Limit(Chassis_t* My_Chassis)
{
	 
	float F_acc, Fw, Ff_cal, Ff_max;
	static float limit_coe_arr[10] = {0.f, 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f};
	float limit_coe = 1.f;
	
	if(My_Chassis->target->yaw_v*My_Chassis->target->sd1 > 0)
	{
		if(My_Chassis->target->sd1 > 0)
		Ff_max = My_Chassis->Leg_Unit[L_Leg]->force->F_support * 0.2f;
		else
		Ff_max = My_Chassis->Leg_Unit[L_Leg]->force->F_support * 0.06f;
		F_acc = My_Chassis->Posture->info->x_world * mw;
		Fw = mw*My_Chassis->Leg_Unit[L_Leg]->Straight->info->sd1*My_Chassis->Posture->info->yaw_v;
		arm_sqrt_f32((F_acc*F_acc + Fw*Fw), &Ff_cal);
		if(Ff_cal > Ff_max)
		{
			limit_coe = limit_coe_arr[binarySearchClosestLess(limit_coe_arr, 10, Ff_max, Ff_cal)];
//			My_Chassis->target->yaw_v *= limit_coe;
			My_Chassis->target->sd1 = My_Chassis->Leg_Unit[L_Leg]->Straight->info->sd1*limit_coe;
		}
	}
	else if(My_Chassis->target->yaw_v*My_Chassis->target->sd1 < 0)
	{
		if(My_Chassis->target->sd1 > 0)
		Ff_max = My_Chassis->Leg_Unit[L_Leg]->force->F_support * 0.2f;
		else
		Ff_max = My_Chassis->Leg_Unit[L_Leg]->force->F_support * 0.06f;
		F_acc = My_Chassis->Posture->info->x_world * mw;
		Fw = mw*My_Chassis->Leg_Unit[R_Leg]->Straight->info->sd1*My_Chassis->Posture->info->yaw_v;
		arm_sqrt_f32((F_acc*F_acc + Fw*Fw), &Ff_cal);
		if(Ff_cal > Ff_max)
		{
			limit_coe = limit_coe_arr[binarySearchClosestLess(limit_coe_arr, 10, Ff_max, Ff_cal)];
//			My_Chassis->target->yaw_v *= limit_coe;
			My_Chassis->target->sd1 = My_Chassis->Leg_Unit[R_Leg]->Straight->info->sd1*limit_coe;
		}
	}
}

static int binarySearchClosestLess(float arr[], int size, float target, float mean)
{
	int low = 0;
	int high = size - 1;
	int result = 0;  // 用于记录满足条件的最后一个索引

	while (low <= high)
	{
		int mid = low + (high - low) / 2;  // 防止溢出

		// 如果当前元素小于 target，则记录下索引，并向右边继续查找更大的数
		if ((arr[mid]*mean) < target)
		{
			result = mid;
			low = mid + 1;
		}
		// 如果 arr[mid] 大于等于 target，则需要向左侧寻找
		else
		{
			high = mid - 1;
		}
	}
return result;
}



static void My_Sd_Position_Nonlinear_Fix(Chassis_t* My_Chassis)
{
	/*
	phi1为前关节，水平角度为180度，角度往上限位方向递增
	
	phi4为后关节，水平角度为0度，角度往上限位方向递减
	*/
	
	float phi1_gate = 0.f, phi4_gate = 0.f;
	

	phi1_gate = L_PHI1_UP_ANGLE - LIMIT_RANGE;
	phi1_gate=half_cycle(phi1_gate,360.f);
	phi4_gate = L_PHI4_UP_ANGLE + LIMIT_RANGE;
	phi4_gate=half_cycle(phi4_gate,360.f);
	
	//右腿
	float phi1_transformed;
	if(My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->phi1_<0)
	{
		phi1_transformed=360.f+My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->phi1_;
	}
	if( phi1_transformed> phi1_gate)
	{
		My_Chassis->Leg_Unit[R_Leg]->force->Sd_F_Limit_Tor_Fix = ((phi1_transformed - phi1_gate) / LIMIT_RANGE) * SD_POS_FIX_TOR_K;
	}
	else
	{
		My_Chassis->Leg_Unit[R_Leg]->force->Sd_F_Limit_Tor_Fix = 0.f;
	}
	
	if(My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->phi4_ < phi4_gate)
	{
		My_Chassis->Leg_Unit[R_Leg]->force->Sd_B_Limit_Tor_Fix = ((My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->phi4_ - (phi4_gate)) / LIMIT_RANGE) * SD_POS_FIX_TOR_K;
	}
	else
	{
		My_Chassis->Leg_Unit[R_Leg]->force->Sd_B_Limit_Tor_Fix = 0.f;
	}
	
	//左腿
	if(My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->phi1_<0)
	{
		phi1_transformed=360.f+My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->phi1_;
	}
	if(phi1_transformed > phi1_gate)
	{
		My_Chassis->Leg_Unit[L_Leg]->force->Sd_F_Limit_Tor_Fix = ((phi1_transformed - phi1_gate) / LIMIT_RANGE) * SD_POS_FIX_TOR_K;
	}
	else
	{
		My_Chassis->Leg_Unit[L_Leg]->force->Sd_F_Limit_Tor_Fix = 0.f;
	}
	
	if(My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->phi4_ < phi4_gate)
	{
		My_Chassis->Leg_Unit[L_Leg]->force->Sd_B_Limit_Tor_Fix = ((My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->phi4_ - (phi4_gate)) / LIMIT_RANGE) * SD_POS_FIX_TOR_K;
	}
	else
	{
		My_Chassis->Leg_Unit[L_Leg]->force->Sd_B_Limit_Tor_Fix = 0.f;
	}
	
}


//static void Chassis_Motor_Target_Angle(Chassis_t* My_Chassis)
//{
//	Four_Bar_Link_t* My_L_Link = &My_Chassis->Link[L_Link];
//	Four_Bar_Link_t* My_R_Link = &My_Chassis->Link[R_Link];
//	Straight_Leg_t* My_Leg = My_Chassis->Leg;
//	
//	My_L_Link->angle_cal(My_L_Link);
//	My_R_Link->angle_cal(My_R_Link);
//	
//	My_Chassis->Sd->motor[2]->tx_info->target_angle = R_F_HORIZON_ANGLE+R_F_TIME*My_R_Link->info->angle->target_phi1;
//	My_Chassis->Sd->motor[3]->tx_info->target_angle = R_B_HORIZON_ANGLE+R_B_TIME*My_R_Link->info->angle->target_phi4;
//	My_Chassis->Sd->motor[1]->tx_info->target_angle = L_F_HORIZON_ANGLE+L_F_TIME*My_L_Link->info->angle->target_phi1;
//	My_Chassis->Sd->motor[0]->tx_info->target_angle = L_B_HORIZON_ANGLE+L_B_TIME*My_L_Link->info->angle->target_phi4;
//	
//	if(Balance.command->chassis->LQR_Model == false)
//	{
//		My_Chassis->Sd->motor[2]->tx_info->Kp = 20;
//		My_Chassis->Sd->motor[2]->tx_info->Kd = 1;
//		My_Chassis->Sd->motor[1]->tx_info->Kp = 20;
//		My_Chassis->Sd->motor[1]->tx_info->Kd = 1;
//		My_Chassis->Sd->motor[0]->tx_info->Kp = 20;
//		My_Chassis->Sd->motor[0]->tx_info->Kd = 1;
//		My_Chassis->Sd->motor[3]->tx_info->Kp = 20;
//		My_Chassis->Sd->motor[3]->tx_info->Kd = 1;
//		My_Leg->T_info->T_mat_storage[Sd_L] = 0;
//		My_Leg->T_info->T_mat_storage[Sd_R] = 0;
//	}
//	else
//	{
//		My_Chassis->Sd->motor[2]->tx_info->Kp = 0;
//		My_Chassis->Sd->motor[2]->tx_info->Kd = 0;
//		My_Chassis->Sd->motor[1]->tx_info->Kp = 0;
//		My_Chassis->Sd->motor[1]->tx_info->Kd = 0;
//		My_Chassis->Sd->motor[0]->tx_info->Kp = 0;
//		My_Chassis->Sd->motor[0]->tx_info->Kd = 0;
//		My_Chassis->Sd->motor[3]->tx_info->Kp = 0;
//		My_Chassis->Sd->motor[3]->tx_info->Kd = 0;
//	}
//}
float run = 0.1f;
float run_add = 0.1f;
float stop = 0.1f;
float stop_add = 0.1f;
static void My_Chassis_KEY_Input(void)
{
	Chassis_Rc_Input_t* rc_input = Chassis.rc_input;
	
	//平移
	if(rc_sensor.info->W.status == short_press || rc_sensor.info->W.status == long_press || rc_sensor.info->W.status == release_to_press)
	{
		rc_input->w_now = (float)rc_sensor.info->W.cnt;
	}
	else if(rc_sensor.info->W.status == press_to_release)
	{
		rc_input->w_now = D_Board_Tx_Pkt.v_x*KEY_W_CNT_MAX/(float)MAX_STRAIGHT_SPEED;;
	}
	else if(rc_sensor.info->W.status == release)
	{
		if(rc_input->w_now > 0)
		{
			rc_input->w_now -= 2;
			if(rc_input->w_now < 0)
				rc_input->w_now = 0;
		}
		else
		{
			rc_input->w_now = 0;
		}
	}
	
	if(rc_sensor.info->S.status == short_press || rc_sensor.info->S.status == long_press || rc_sensor.info->S.status == release_to_press)
	{
		rc_input->s_now = (float)rc_sensor.info->S.cnt;
	}
	else if(rc_sensor.info->S.status == press_to_release)
	{
		rc_input->s_now = -D_Board_Tx_Pkt.v_x*KEY_S_CNT_MAX/(float)MAX_STRAIGHT_SPEED;;
	}
	else if(rc_sensor.info->S.status == release)
	{
		if(rc_input->s_now > 0)
		{
			rc_input->s_now -= 2;
			if(rc_input->s_now < 0)
				rc_input->s_now = 0;
		}
		else
		{
			rc_input->s_now = 0;
		}
	}
	//平移
	
	if(rc_sensor.info->D.status == short_press || rc_sensor.info->D.status == long_press || rc_sensor.info->D.status == release_to_press)
	{
		rc_input->d_now = (float)rc_sensor.info->D.cnt;
	}
	else if(rc_sensor.info->D.status == press_to_release)
	{
		rc_input->d_now = -D_Board_Tx_Pkt.v_y*KEY_D_CNT_MAX/(float)MAX_STRAIGHT_SPEED;;
	}
	else if(rc_sensor.info->D.status == release)
	{
		if(rc_input->d_now > 0)
		{
			rc_input->d_now -= 2;
			if(rc_input->d_now < 0)
				rc_input->d_now = 0;
		}
		else
		{
			rc_input->d_now = 0;
		}
	}
	
	if(rc_sensor.info->A.status == short_press || rc_sensor.info->A.status == long_press || rc_sensor.info->A.status == release_to_press)
	{
		rc_input->a_now = (float)rc_sensor.info->A.cnt;
	}
	else if(rc_sensor.info->A.status == press_to_release)
	{
		rc_input->a_now = -D_Board_Tx_Pkt.v_y*KEY_A_CNT_MAX/(float)MAX_STRAIGHT_SPEED;;
	}
	else if(rc_sensor.info->A.status == release)
	{
		if(rc_input->a_now > 0)
		{
			rc_input->a_now -= 2;
			if(rc_input->a_now < 0)
				rc_input->a_now = 0;
		}
		else
		{
			rc_input->a_now = 0;
		}
	}
	
	rc_input->a_now = Lowpass(rc_input->a_last,(float)rc_sensor.info->A.cnt, 0.4f);
	rc_input->a_last = rc_input->a_now;
	
	//腿长
	//平移
	rc_input->d_now = Lowpass(rc_input->d_last,(float)rc_sensor.info->D.cnt, 0.4f);
	rc_input->d_last = rc_input->d_now;
}


#define CHASSIS_MAX_POWER_BUFFER  		(40.f) //功率限制阈值，当buffer小于40时开始做限制
#define CHASSIS_MID_POWER_BUFFER  		(20.f) 
#define CHASSIS_LOW_POWER_BUFFER  		(10.f) //各个值经简单调整，大量测试后没问题既可
#define CHASSIS_DANGER_POWER_BUFFER		(5.f)  
static float My_Chassis_Power_Limit(void)
{
	float buff = 0.f;
	
	float limit_k = 1.f;
	
	if(judge.status->status == DEV_ONLINE)
	{
		buff = judge.info->power_heat_data.buffer_energy;
	}
	
	/*功率限制*/
	if(buff > CHASSIS_MAX_POWER_BUFFER)//飞坡完成后,buffer会自动变高并停留几秒
	{
		buff = CHASSIS_MAX_POWER_BUFFER;//无限制 大于40
		limit_k = 1;
	}
	else if(buff >= CHASSIS_MID_POWER_BUFFER)//一级限制 20~40
	{
		limit_k = buff / CHASSIS_MAX_POWER_BUFFER;
	}
	else if(buff >= CHASSIS_LOW_POWER_BUFFER)//二级限制 10~20
	{
		limit_k = buff / CHASSIS_MAX_POWER_BUFFER;
	}
	else//三级限制 buffer小于10
	{
		  limit_k = 0.15f;

	}
	
	if(Balance.Flag->Fly_Flag == true || Balance.Flag->Reserve_Fly_Flag == true)
	{
		limit_k = 1;
	}
	
	
	return limit_k;

}


float a = 0,Nf = 0,kk;
float Tw_Enable = 7.f;
float Tp_Big;
float ttt1,ttt2,ttt3,ttt4;
float KKK;
static void Chassis_Power_Limit(Chassis_t* My_Chassis)
{
	static float power_limit = 60.f;
	Tw_Enable = judge.info->power_heat_data.buffer_energy/24.f*_3508_TORQUE_CONSTANT;
//		Tw_Enable = judge.info->/24.f*_3508_TORQUE_CONSTANT;
//	
//	KKK=judge.info->power_heat_data.buffer_energy/60.f;
//	KKK = sqrt(KKK);
//	
	Leg_Unit_t* Leg_Unit;

	if(fabs(My_Chassis->Leg_Unit[R_Leg]->Straight->u->u_mat_storage[Tp]) >= fabs(My_Chassis->Leg_Unit[L_Leg]->Straight->u->u_mat_storage[Tp]))
	{
		Tp_Big = fabs(My_Chassis->Leg_Unit[R_Leg]->Straight->u->u_mat_storage[Tp]);
		Leg_Unit = My_Chassis->Leg_Unit[R_Leg];
	}
	else
	{
		Tp_Big = fabs(My_Chassis->Leg_Unit[L_Leg]->Straight->u->u_mat_storage[Tp]);
		Leg_Unit = My_Chassis->Leg_Unit[L_Leg];
	}
	
//	Nf = (float)(Tw_Enable / WHEEL_RADIUS) - (float)((2*Tp_Big / Leg_Unit->Link->info->length->l0) * abs(arm_cos_f32(Leg_Unit->Link->info->angle->vir_phi0-My_Chassis->Posture->info->pitch))) \
//		                                    - (float)(mb*g*(Leg_Unit->Link->info->length->l0)*abs(arm_sin_f32(Leg_Unit->Link->info->angle->vir_phi0-My_Chassis->Posture->info->pitch))) \
//																				- (float)(mb*g*abs(arm_cos_f32(Leg_Unit->Link->info->angle->vir_phi0-My_Chassis->Posture->info->pitch))*abs(arm_sin_f32(Leg_Unit->Link->info->angle->vir_phi0-My_Chassis->Posture->info->pitch)));
	
	Nf = (float)(Tw_Enable / WHEEL_RADIUS) - (float)((2*Tp_Big / Leg_Unit->Link->info->length->l0) * abs(arm_cos_f32(Leg_Unit->Link->info->angle->vir_phi0))) \
		                                    - (float)(mb*g*(Leg_Unit->Link->info->length->l0)*abs(arm_sin_f32(Leg_Unit->Link->info->angle->vir_phi0))) \
																				- (float)(mb*g*abs(arm_cos_f32(Leg_Unit->Link->info->angle->vir_phi0))*abs(arm_sin_f32(Leg_Unit->Link->info->angle->vir_phi0)));																			
	
	ttt1 = (float)(Tw_Enable / WHEEL_RADIUS);
  ttt2 = (float)((2*Tp_Big / Leg_Unit->Link->info->length->l0) * abs(arm_cos_f32(Leg_Unit->Link->info->angle->vir_phi0)));
  ttt3 = (float)(mb*g*(Leg_Unit->Link->info->length->l0)*abs(arm_sin_f32(Leg_Unit->Link->info->angle->vir_phi0)));
  ttt4 = (float)(mb*g*abs(arm_cos_f32(Leg_Unit->Link->info->angle->vir_phi0))*abs(arm_sin_f32(Leg_Unit->Link->info->angle->vir_phi0)));																		
  																				
																				
//	kk = arm_sin_f32(Leg_Unit->Link->info->angle->vir_phi0-My_Chassis->Posture->info->pitch);
  kk = arm_sin_f32(Leg_Unit->Link->info->angle->vir_phi0);																				
	
	a = (float)Nf/m_all;//质量得换整车的
	
	My_Chassis->target->velocity_limit = My_Chassis->target->velocity_limit + a*TIME_STEP;
	
	if(My_Chassis->target->velocity_limit >= MAX_STRAIGHT_SPEED)
	{
		My_Chassis->target->velocity_limit = MAX_STRAIGHT_SPEED;
	}
	
	if(My_Chassis->target->velocity_limit <= 0.4)//阈值得调
	{
		My_Chassis->target->velocity_limit = 0.4;
	}
	
//	My_Chassis->target->velocity_limit *= KKK;
}


/*氮气弹簧动态前馈*/
void My_Spring_Former_Input_Cal(Link_info_t* R_Link,Link_info_t* L_Link)
{
	static float Alpha_R,Belta_R;//Alpha是腿交点,Belta是腿延长线交点
	static float Alpha_L,Belta_L;
	static float Spring_Force = 40 * g;//400N
	
	Alpha_R = PI - R_Link->angle->phi3 + R_Link->angle->phi4;
	Belta_R = R_Link->angle->phi2 - R_Link->angle->phi4;
	
	Alpha_L = PI - L_Link->angle->phi3 + L_Link->angle->phi4;
	Belta_L = L_Link->angle->phi2 - L_Link->angle->phi4;
	
	R_Link->force->Spring_T_Feed_Front = (Spring_Force * 0.06 * 0.095 / 0.116) * arm_sin_f32(Alpha_R + 0.26179938f) *arm_cos_f32(Belta_R);
	R_Link->force->Spring_T_Feed_Back = Spring_Force * arm_sin_f32(1.22173047f) * 0.04715;
	
	L_Link->force->Spring_T_Feed_Front = (Spring_Force * 0.06 * 0.095 / 0.116) * arm_sin_f32(Alpha_L + 0.26179938f) *arm_cos_f32(Belta_L);
	L_Link->force->Spring_T_Feed_Back = Spring_Force * arm_sin_f32(1.22173047f) * 0.04715;
	
}


float Chassis_S_Turn_sdl_update(Chassis_t* My_Chassis)
{
	float angle_err,tar_x,tar_y,turn_x,turn_y,tar_sdl;
	static float turn_k = 0;
	
	angle_err = Y_ZERO_ANGLE - gimbal.yaw->rx_info->motor_angle;
	
	if(fabs(angle_err) >= PI)
	{
		angle_err -= sgn(angle_err) * 2 *PI;
	}
	
	if(Balance.ctrl == RC_CTRL)
	{
		tar_x = -My_Chassis->rc_input->ch3_now / 660.f * 1.8f;
		tar_y = -My_Chassis->rc_input->ch2_now / 660.f * 1.8f;
	}
	else if(Balance.ctrl == KEY_CTRL)
	{
		tar_x = -1.f * ((float)My_Chassis->rc_input->w_now - (float)My_Chassis->rc_input->s_now )/ (float)rc_sensor.info->S.cnt_max* 1.8f;
		tar_y = -1.f * ((float)My_Chassis->rc_input->d_now - (float)My_Chassis->rc_input->a_now )/ (float)rc_sensor.info->A.cnt_max* 1.8f;
		
	}
	turn_x = tar_x * arm_sin_f32(angle_err);
	turn_y = tar_y * arm_cos_f32(angle_err);
	
	tar_sdl = turn_x + turn_y;
	
	if(judge.info->power_heat_data.buffer_energy <= 55)
	{
		turn_k = (float)judge.info->power_heat_data.buffer_energy/60.f;
		tar_sdl *= turn_k;
	}
	
	
	
	return tar_sdl;
}


void Fry_detect(Chassis_t* My_Chassis)
{
	
	
	
	
	
}
