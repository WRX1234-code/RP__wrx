#ifndef __CHASSIS_H
#define __CHASSIS_H

/* Includes ------------------------------------------------------------------*/
#include "arm_math.h"
#include "rp_config.h"
#include "chassis_motor.h"
#include "Chassis_Posture.h"
#include "Link_Instance.h"
#include "Straight_Instance.h"
#include "PID_Instance.h"
#include "Balance.h"

/* Exported macro ------------------------------------------------------------*/

//使用氮气弹簧前馈
#define SPRING_USED    


/* Exported types ------------------------------------------------------------*/

/*底盘模式*/
typedef enum
{
	C_Sleep,				//卸力
	C_Init,         //初始化
	C_Follow,				//跟随
	C_Boss,					//控制底盘
	C_Turn,         //普通小陀螺
	C_Test,			    //测试模式,分电控调试跟视觉调试
	C_Rescue,		    //倒地二轮车
	C_Lob,          //吊射
	C_Jump,         //跳跃
	C_Knee_Strike,  //磕膝上台阶
	C_Fly           //飞坡
}Chassis_Mode_e;


typedef enum
{
	Front,
	Behind,
	L_Front,
	R_Front,
}Chassis_Init_State_e;

typedef enum
{
	Chassis_reset_NO,
	Chassis_reset_OK,
}Chassis_reset_state_e;

typedef struct Chassis_reset_state_struct_t
{
	Chassis_reset_state_e reset_state;
	uint16_t reset_cnt;
}Chassis_reset_state_t;

/*底盘设备状态*/
typedef struct Chassis_state_struct_t
{
	dev_work_state_t sd_state;
	
	dev_work_state_t wheel_state;
}Chassis_state_t;

/*底盘目标值*/
typedef struct Chassis_Target_struct_t
{
	uint8_t stuck_shorten_leg_flag;
	
	bool hard_turn_flag;
	
	float thetal_l;
	
	float thetal_r;
	
	float vir_phi0_l;
	
	float vir_phi0_r;
	
	float vir_phi0d1_l;
	
	float vir_phi0d1_r;
	
	float thetald1;
	
	float s;
	
	float sd1;
	
	float thetab;
	
	float thetabd1;
	
	float velocity_limit;
	
	float velocity_y;//小陀螺时使用
	
	float yaw_v;

	float yaw;
	
	float roll;
	
	float leg_length_r;
	float leg_length_l;
	
}Chassis_Target_t;

/*遥控器输入*/
typedef struct Chassis_Rc_Input_struct_t
{
	int16_t ch3_now;//前后
	
	int16_t ch3_last;
	
	int16_t ch2_now;//腿长
	
	int16_t ch2_last;
	
	int16_t ch1_now;//头pitch
	
	int16_t ch1_last;
	
	int16_t ch0_now;//偏航
	
	int16_t ch0_last;
	
	float w_now;
	
	float w_last;
	
	float a_now;
	
	float a_last;
	
	float s_now;
	
	float s_last;
	
	float d_now;
	
	float d_last;
	
}Chassis_Rc_Input_t;

typedef struct
{
  /*竖直力begin*/
  float F_gravity;//重力补偿力
  
  float F_inertial;//侧向惯性补偿力
  
  float F_roll;//roll轴补偿力
  
  float F;//保持腿长力,pid,伸腿为正
	
  float F_bl_target;//合力,F+F_roll+F_inertial+F_gravity
	/*竖直力end*/
	
	/*关节力begin*/	
	float Tp_sync;//双腿协调
	float Tp_LQR;
	float Tp_vir_phi0_;//换成-180-180
	float Tp_vir_phi0_d1;
	float Tp_target;
	
	/*关节力end*/	
	
	/*驱动轮力矩begin*/	
	float Tw_turn;
	float Tw_LQR;
	float Tw_target;
	/*驱动轮力矩end*/	
	
	/*关节限位补偿力，防止撞限位*/
	float Sd_F_Limit_Tor_Fix;
	float Sd_B_Limit_Tor_Fix;
	
	/*关节电机实际输出力矩*/
	float Sd_F_Torque;
	float Sd_B_Torque;
	
	float F_support;//支持力
	
}Leg_force_t;

typedef struct
{
	Link_t* Link;//五连杆解算、VMC
	
	Straight_Leg_t* Straight;//直腿模型
	
	Leg_force_t* force;
	
	uint16_t off_ground_cnt;
	
	uint8_t off_ground;
	
}Leg_Unit_t;

typedef enum
{
	J_IDLE,
	J_COMPRESS,//压缩
	J_EXTEND, //伸腿
	J_RETRACT,//收腿
	J_PRE_LANDING,//伸腿准备落地
	J_LANDING,//缓冲
	Jump_Step_Num,
	
}Jump_Step_e;

typedef struct
{
	bool r_offground;
	bool l_offground;
	float l0_average;
	float Minimum_l0_range;
	float Max_l0_range;
	float Landing_l0_range;
	uint16_t jump_tick;
	uint16_t COMPRESS_tick;
	uint16_t EXTEND_tick;
	uint16_t RETRACT_tick;
	uint16_t PRE_LANDING_tick;
	uint16_t LANDING_tick;
	
	uint16_t Max_COMPRESS_tick;
	uint16_t Max_EXTEND_tick;
	uint16_t Max_RETRACT_tick;
	uint16_t Max_PRE_LANDING_tick;
	uint16_t Max_LANDING_tick;
	
	float IDLE_length_kp;
	float IDLE_length_speed_kp;
	float IDLE_length_outmax;
	float IDLE_length_speed_outmax;
	
	float COMPRESS_length_kp;
	float EXTEND_length_kp;
	float RETRACT_length_kp;
	float PRE_LANDING_length_kp;
	float LANDING_length_kp;
	float LANDING_length_speed_kp;
	
	Jump_Step_e jump_step;
}Chassis_Jump_t;

typedef enum
{
	Knee_IDLE,
	Knee_Stand_High,
	Knee_RETRACT,
	Knee_Strike_Num,
}Knee_Strike_Step_e;

typedef struct
{
	float Minimum_l0_range;
	float Max_l0_range;
	float Max_Stand_High_tick;
	float Max_RETRACT_tick;
	
	float l0_average;
	float thetal_average;
	float Stand_High_tick;
	float RETRACT_tick;
	float thetal_threshold;
	float IDLE_length_r_kp;
	float IDLE_length_l_kp;
	float STAND_length_kp;
	float RETRACT_length_kp;
	
	Knee_Strike_Step_e step;
	
}Chassis_Knee_Strike_t;

typedef struct
{
	float l0_length_kp;
	float l0_length_speed_kp;
	float l0_length_outmax;
	float l0_length_speed_outmax;
}Chassis_pid_init_parament_t;//存储最开始的pid参数


typedef enum{
	R_IDIE,
	R_LEG_RESTRACT,
	R_LEG_OFF,
  R_STUMBLE,
	R_RECLINE,
	
}Rescue_State_e;



typedef struct{
	uint8_t first_in_flag;
	uint8_t is_rescue;
	uint16_t restrict_cnt;
	uint16_t leg_off_cnt;
	uint16_t stumble_cnt;
	uint16_t recline_cnt;
	uint8_t stumble_proc;
	uint8_t recline_proc;
	float yaw_save_tar;
	float yaw_save_range;
	uint16_t yaw_save_cnt;
	uint16_t yaw_cnt_max;
	
	Rescue_State_e state;
	Rescue_State_e last_state;

}Chassis_Rescue_t;



/*底盘结构体*/
typedef struct Chassis_struct_t
{
	Chassis_Mode_e mode;
	Chassis_Mode_e last_mode;
	
	Chassis_state_t* state;
	Chassis_reset_state_t* reset_struct;
	
	Leg_Unit_t* Leg_Unit[Leg_Num];
	
	Chassis_Target_t* target;
	
	Chassis_Rc_Input_t* rc_input;
	
	uint16_t damping_delay_cnt;
	
	Chassis_Pid_t* chassis_PID;
	
	Chassis_Init_State_e init_state;
	
	Chassis_Posture_t* Posture;
	
	Chassis_Jump_t* jump_info;
	
	Chassis_Knee_Strike_t* knee_strike_info;
	
	Chassis_Rescue_t* rescue_info;
	
	Motor_RM_Group_t* Wheel;

	Motor_DM_Group_t* Sd;
	
	void (*Init)(struct Chassis_struct_t* My_Chassis);//初始化函数
	
	void (*heartbeat)(struct Chassis_struct_t* My_Chassis);
	
	void (*data_update)(struct Chassis_struct_t* My_Chassis);
	
	void (*status_react)(struct Chassis_struct_t* My_Chassis);
	
	void (*ctrl)(struct Chassis_struct_t* My_Chassis);
	
	void (*work)(struct Chassis_struct_t* My_Chassis);
	Chassis_pid_init_parament_t* pid_init_parament[Leg_Num];
	
}Chassis_t;

/* Exported functions --------------------------------------------------------*/
extern Chassis_t Chassis;
extern float My_Wheel_Sb;
extern float My_Imu_Sb;
extern float My_filter_Sb;
extern float My_filter_Sb2;
/* Servo functions */

#endif
