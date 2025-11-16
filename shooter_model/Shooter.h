#ifndef __SHOOTER_H
#define __SHOOTER_H

#include "stm32f4xx.h"


#define   CLOSE_SHOOTER                                    //该操作关闭发射机构的保险
#define   OPEN_SHOOTER                                     //该操作打开发射机构的保险

#define   SWITCH_SIMGLE_SHOT                               //该操作将模式切换成单发
#define   SWITCH_BURST                                     //该模式将操作切换成连发

#define   RISING_EDGE_TRIGGER                              //该操作使单发开火
#define   HIGH_LEVEL_TRIGGER                               //该操作使连发开火

#define   BURST_MODE                                       //连发模式分角度环模式 0，跟速度环模式 1
#define   DIAL_MEC_LIMIT                                   //拨盘无机械限位 0，有机械限位 1
  
	
#define   DIAL_TYPE                                        //拨盘电机类型
#define   ONESHOT_ANGLE                                    //拨盘正转拨一颗弹丸，电机所转过的角度
#define   DIAL_RESET_SPEED                                 //拨盘复位时的电机转速，低速
#define   DIAL_RELOAD_SPEED                                //速度环连发模式下的拨盘电机补弹速度
#define   DIAL_WORK_TIME_MAX                               //拨盘最大工作时间，超过该时间拨盘睡眠
#define   DIAL_FIRING_PERIOD                               //角度环连发模式下的拨盘电机补弹周期

#define   DIAL_ANGLE_DATA_TYPE          uint16_t           //角度数据类型
#define   DIAL_SPEED_DATA_TYPE          int16_t            //速度数据类型
#define   DIAL_CURRENT_DATA_TYPE        int16_t            //电流数据类型
#define   DIAL_ANGLE_SUM_DATA_TYPE      int64_t            //角度总和数据类型


#define   FRIC_TYPE                                        //摩擦轮电机类型       
#define   FRIC_NUM                      6                  //摩擦轮数量，分二摩 2，三摩 3，六摩 6，     
#define   FRIC_FIRST_STAGE_SPEED                           //六摩的第一级摩擦轮目标速度，二，三摩目标速度     
#define   FRIC_SECOND_STAGE_SPEED                          //六摩的第二级摩擦轮目标速度       
#define   FRIC_INIT_SPEED                                  //摩擦轮初始化状态的速度         
#define   FRIC_BLOCK_SPEED                                 //摩擦轮堵转状态下的速度，负值   
#define   FRIC_TEMP_MAX                                    //摩擦轮最高温度

#define   FRIC_ANGLE_DATA_TYPE          uint16_t           //角度数据类型             
#define   FRIC_SPEED_DATA_TYPE          int16_t            //速度数据类型                  
#define   FRIC_CURRENT_DATA_TYPE        int16_t            //电流数据类型           
#define   FRIC_ANGLE_SUM_DATA_TYPE      int32_t            //角度总和数据类型              


typedef enum{
	FRIC_B_L = 0,                     //六摩第一级左边摩擦轮，二，三摩左边摩擦轮
	FRIC_B_R,                         //六摩第一级右边摩擦轮，二，三摩右边摩擦轮
	FRIC_B_UP,                        //六摩第一级上边摩擦轮，三摩上边摩擦轮
	
	FIRC_F_L = 3,                     //六摩第二级左边摩擦轮        
	FRIC_F_R,                         //六摩第二级右边摩擦轮       
	FRIC_F_UP,                        //六摩第二级上边摩擦轮   
	
	FRIC_LIST,

}Fric_list_e;


typedef struct{
	float   target;
	float   measure;
	float   kp;
	float   ki;
	float   kd;
	float   err;
	float   last_err;                   //保存上一次的误差
	float   pout;
	float   iout;
	float   dout;
	float   integral;
	float   integral_max;               //积分上限
	float   out;
	float   out_max;                    //输出上限
	float   filter_value;               //低通滤波系数
	
}Basic_Pid_t;


typedef struct{
	Basic_Pid_t     inner;              //双环PID内环
	Basic_Pid_t     outer;              //双环PID外环

}Double_Pid_t;


typedef struct{
	Double_Pid_t     angle;             //角度双环
  Double_Pid_t     speed;             //速度单环
  
}Motor_Pid_t;


typedef enum{
	GM_6020,                            
	M_3508,
	M_2006,
	DM_J4310,
	KT_4005,

}Shoot_Motor_Type_e;


typedef struct{
	DIAL_ANGLE_DATA_TYPE         angle;             //角度
  DIAL_SPEED_DATA_TYPE         speed;             //速度
  DIAL_CURRENT_DATA_TYPE       current;           //电流
	DIAL_ANGLE_SUM_DATA_TYPE     angle_sum;         //角度和
	uint8_t                      temperature;       //温度
	
}Dial_Rx_Info_t;


typedef struct{
	FRIC_ANGLE_DATA_TYPE         angle;             //角度
  FRIC_SPEED_DATA_TYPE         speed;             //速度
  FRIC_CURRENT_DATA_TYPE       current;           //电流
	FRIC_ANGLE_SUM_DATA_TYPE     angle_sum;         //角度和
	uint8_t                      temperature;       //温度，部分电机无法读取如2006
	
}Fric_Rx_Info_t;


typedef struct{
  DIAL_CURRENT_DATA_TYPE     torque;           //拨盘电机要发送的扭矩
	
}Dial_Tx_Cmd_t;


typedef struct{
  FRIC_CURRENT_DATA_TYPE     torque;           //摩擦轮电机要发送的扭矩
 
}Fric_Tx_Cmd_t;


typedef struct{
	Fric_Rx_Info_t     info;
  Fric_Tx_Cmd_t      cmd;
  Motor_Pid_t        pid;
	
}Fric_Xfer_t;


typedef struct{
	Shoot_Motor_Type_e           type;
	Dial_Rx_Info_t               info;            
  Dial_Tx_Cmd_t                cmd;             
  Motor_Pid_t                  pid;
	
}Dial_t;


typedef struct{
	Shoot_Motor_Type_e       type;
	Fric_Xfer_t              xfer[FRIC_NUM];
	FRIC_SPEED_DATA_TYPE     speed_err[FRIC_NUM];         //摩擦轮速度与目标速度误差
	uint8_t                  temp_err[FRIC_NUM];          //摩擦轮温度与温度上限误差
  
}Fric_t;


typedef enum{
	LOCKED,                              //发射机构关保险，无法射击
	INITING,                             //发射机构开保险后的初始化，摩擦轮转动，拨盘复位
	UNLOCK,                              //发射机构初始化后的开保险状态，可以射击

}Shoot_Safe_State_e;


typedef enum{
	CEASEFIRE,                           //停火模式，对应拨盘的睡眠模式
	SIMGLE_SHOT,                         //单发模式
	BURST,                               //连发模式
	
}Shoot_Work_State_e;


typedef enum{
	RESETING,                            //复位状态，速度环，拨盘有机械限位反转，无机械限位正转
	SLEEP,                               //睡眠模式不工作，不计工作时间
	RELOAD,                              //补弹状态
	RECOIL,                              //退弹状态，堵转时触发

}Dial_Work_State_e;


typedef struct{
	uint8_t   inited_flag;               //初始化完成标志位，未完成为 0，完成为 1
	uint8_t   fire_flag;                 //单发开火标志位，未开火为 0，开火为 1
	uint8_t   firing_flag;               //连发开火标志位，未开火为 0，开火为 1
	uint8_t   dial_block_flag;           //拨盘堵转标志位，未堵转为 0，堵转为 1
	uint8_t   fric_block_flag;           //摩擦轮堵转标志位，未堵转为 0，堵转为 1 
	uint8_t   load_flag;                 //补弹标志位，未补弹未 0，补弹为 1
	uint8_t   fric_speed_flag;           //摩擦轮速度正常标志位，正常为 1，不正常为 0
	uint8_t   fric_temp_flag;            //摩擦轮温度标志位，温度正常为 1，不正常为 0

}Shoot_Flag_t;


typedef struct{
	DIAL_ANGLE_SUM_DATA_TYPE     dial_angle_sum_target;          //拨盘角度和目标
  DIAL_SPEED_DATA_TYPE         dial_speed_target;              //拨盘速度目标
  FRIC_SPEED_DATA_TYPE         fric_speed_target;              //摩擦轮速度目标
	
}Shoot_Motor_Target_t;


typedef struct{
	DIAL_CURRENT_DATA_TYPE     dial_output;                       //拨盘电机输出
	FRIC_CURRENT_DATA_TYPE     firc_output[FRIC_NUM];             //摩擦轮电机各自输出
	
}Shoot_Motor_Output_t;


typedef struct{
	Dial_t                   dial;                                //拨盘基本配置
	Fric_t                   fric;                                //摩擦轮各自基本配置
	Shoot_Safe_State_e       shoot_safe_state;
  Shoot_Work_State_e       shoot_work_state;
  Dial_Work_State_e        dial_work_state;
	Shoot_Flag_t             shoot_flag;
	Shoot_Motor_Target_t     shoot_motor_target;
	Shoot_Motor_Output_t     shoot_motor_output;
	
}Shoot_t;


Motor_Pid_t dial_pid={
	.speed={
		.inner={
			.kp = 0,
		  .ki = 0,
		  .kd = 0,
		  .integral_max = 0,
		  .out_max = 0,
		  .filter_value = 0,
		}
	},
	
	.angle={
	  .inner={
		  .kp = 0,
		  .ki = 0,
		  .kd = 0,
		  .integral_max = 0,
		  .out_max = 0,
		  .filter_value = 0,
		},
		
		.outer={
		  .kp = 0,
		  .ki = 0,
		  .kd = 0,
		  .integral_max = 0,
		  .out_max = 0,
		  .filter_value = 0,
	  },
	}
	
};


Motor_Pid_t fric_pid={
	.speed={
		.inner={
			.kp = 0,
		  .ki = 0,
		  .kd = 0,
		  .integral_max = 0,
		  .out_max = 0,
		  .filter_value = 0,
		}
	},
	
};


#endif
