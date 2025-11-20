/*---------------------------------------------------------------------------------
 *  @file       shoot_base.h
 *  @brief      发射机构模块对外接口
 *  @author     WRX
 *  @date       
 *  @version    1.0.0
 *  @copyright  
 *-------------------------------------------------------------------------------*/

#ifndef __SHOOT_BASE_H
#define __SHOOT_BASE_H


/**-------------------------------------------------------------------------------
 * 机械坐标系约定
 * -------------------------------------------------------------------------------
 * 视线方向 : 从电机尾部 → 输出轴端看
 * 正方向   : 逆时针（CCW）为正
 * 负方向   : 顺时针（CW）
 * -------------------------------------------------------------------------------
 * 所有与转速、角度、扭矩相关的Info,Cmd接口变量均遵守本约定。
 * 例：
 *   speed_rpm  =  1800  -> 1800 rpm 逆时针
 *   speed_rpm  = -1200  -> 1200 rpm 顺时针
 *-------------------------------------------------------------------------------*/


/*--------------------------------外部头文件引用---------------------------------*/
#include "stm32f4xx.h"

 
/*--------------------------------自定义宏定义-----------------------------------*/

//拨盘
#define  DIAL_MOTOR_TYPE                  M_2006_                  //拨盘电机类型，从Dial_Motor_Type_e里面选

#define  DIAL_MEC_LIMIT                   0                        //拨盘有无机械限位，无为 0，有为 1

#define  DIAL_ANGLE_DATA_TYPE             uint16_t                 //拨盘角度数据类型
#define  DIAL_SPEED_DATA_TYPE             int16_t                  //拨盘速度数据类型
#define  DIAL_CURRENT_DATA_TYPE           int16_t                  //拨盘电流数据类型
#define  DIAL_ANGLE_SUM_DATA_TYPE         int32_t                  //拨盘角度和数据类型

//摩擦轮
#define  FRIC_NUM                         3                        //摩擦轮数量，分六摩 6，三摩 3，二摩 2

#define  FRIC_SPEED_DATA_TYPE             int16_t                  //摩擦轮速度数据类型
#define  FRIC_CURRENT_DATA_TYPE           int16_t                  //摩擦轮电流数据类型

/*----------------------------------枚举定义-------------------------------------*/

/**
 * @brief  拨盘电机类型枚举
 */
typedef enum{
	GM_6020_,
	M_3508_,
	M_2006_,
	DM_J4310_,
	KT_4005_,
	
}Dial_Motor_type_e;

/**
 * @brief  拨盘运动状态枚举
 */
typedef enum{
	RESETING,                         //复位状态
	SLEEP,                            //睡眠状态
	RELOAD,                           //补弹状态
	RECOIL,                           //退弹状态

}Dial_Work_State_e;

/**
 * @brief  摩擦轮运动状态枚举
 */
typedef enum{
	RUN,                               //转动
	STOP,                              //不转
	
}Fric_Work_State_e;

/**
 * @brief  拨盘转动模式枚举
 */
typedef enum{
	DIAL_SPEED,                       //速度环
	DIAL_ANGLE,                       //角度环

}Dial_Mode_e;

/**
* @brief  发射机构开火模式枚举
 */
typedef enum{
	CEASEFIRE,                          //停火模式
	SIMGLE_SHOT,                        //单发模式
	BURST,                              //连发模式

}Shoot_Mode_e;

/**
* @brief  发射机构工作状态枚举
 */
typedef enum{
	LOCKED,                             //关保险状态，无法射击
	INITING,                            //初始化状态，无法射击
	UNLOCK,                             //开保险状态，允许射击

}Shoot_Work_State_e;

/**
* @brief  发射机构摩擦轮区域枚举
 */
#if  FRIC_NUM == 6
typedef enum{
  FRIC_B_UP = 0,                    //第一级上边摩擦轮
	FRIC_B_L,                         //第一级左边摩擦轮
	FRIC_B_R,                         //第一级右边摩擦轮
	
  FRIC_F_UP,                        //第二级上边摩擦轮
	FRIC_F_L,                         //第二级左边摩擦轮        
	FRIC_F_R,                         //第二级右边摩擦轮        
        
  FRIC_LIST,
}Fric_Zone_e; 

#elif FRIC_NUM == 3
typedef enum{
  FRIC_UP = 0,                      //上边摩擦轮
  FRIC_R,                           //左边摩擦轮
  FRIC_L,                           //右边摩擦轮
  
	FRIC_LIST,
}Fric_Zone_e;

#elif FRIC_NUM ==2
typedef enum{
  FRIC_R = 0,                       //左边摩擦轮
  FRIC_L,                           //右边摩擦轮

  FRIC_LIST,
}Fric_Zone_e;

#endif




/*--------------------------------结构体/联合体----------------------------------*/

/**
 * @brief  拨盘数据接收结构体
 */
typedef struct{
	DIAL_ANGLE_DATA_TYPE                 angle;           //角度
	DIAL_SPEED_DATA_TYPE                 speed;           //速度
  DIAL_CURRENT_DATA_TYPE               current;         //电流
	DIAL_ANGLE_SUM_DATA_TYPE             angle_sum;       //角度和
	uint8_t                              temperature;     //温度，部分电机不反馈如2006

}Dial_Rx_Info_t;

/**
 * @brief  摩擦轮数据接收结构体
 */
typedef struct{ 
	FRIC_SPEED_DATA_TYPE                 speed[FRIC_LIST];             //速度
  FRIC_CURRENT_DATA_TYPE               current[FRIC_LIST];           //电流
	uint8_t                              temperature[FRIC_LIST];       //温度，部分电机不反馈如2006

}Fric_Rx_Info_t;


/**
 * @brief  拨盘数据基本配置结构体
 */
typedef struct{
  DIAL_ANGLE_DATA_TYPE          oneshot_angle;            //拨一颗弹，拨盘电机转过的角度
	DIAL_SPEED_DATA_TYPE          reset_speed;              //拨盘电机复位速度，低速
	DIAL_ANGLE_DATA_TYPE          reset_adjust_angle;       //拨盘复位后往回调整的角度，用于寻求最佳射击角度
	DIAL_ANGLE_DATA_TYPE          switch_adjust_angle;      //速度环连发情况下，切换成角度环需要往回调整的角度，保证角度环始终每次转动一整个oneshot_angle
	DIAL_SPEED_DATA_TYPE          reload_speed;             //补弹速度，较高速
	
  Dial_Mode_e                   burst_mode;               //拨盘连发转动模式，分速度环和角度环
	uint8_t                       burst_period;             //拨盘角度环连发周期
  uint8_t                       state_work_time_max;      //拨盘最大工作时间
	
}Dial_Base_Config_t;

typedef struct{
  FRIC_SPEED_DATA_TYPE          speed_target;             //目标速度
	uint8_t                       temp_max;                 //最大温度
  
}Fric_Base_Config_t;

/**
 * @brief  拨盘堵转状态数据配置结构体
 */
typedef struct{
	DIAL_SPEED_DATA_TYPE           speed_max;               //堵转判断最大速度
	DIAL_CURRENT_DATA_TYPE         current_min;             //堵转判断最小电流
	uint8_t block_time_max;                                 //堵转判断最大时间
  uint8_t menage_time_max;                                //

}Motor_Block_Config_t;

/**
 * @brief  拨盘配置总结构体
 */
typedef struct{
	Dial_Base_Config_t           base_config;                //基本配置
	Motor_Block_Config_t         reset_block_config;        //复位堵转配置
  Motor_Block_Config_t         reload_block_config;       //补弹堵转配置

}Dial_Config_t;

/**
 * @brief  摩擦轮配置总结构体
 */
typedef struct{
	Fric_Base_Config_t                  base_config;          //基本配置
	Motor_Block_Config_t                block_config;         //堵转配置
	
}Fric_config_t;


/**
 * @brief  拨盘命令发送结构体
* @note   只发送拨盘状态，模式，目标值，以及视觉需要的拨弹进度
 */
typedef struct{
	Dial_Work_State_e             work_state;             //工作状态
	Dial_Mode_e                   mode;                   //工作模式
  float                         reload_sche;            //拨弹进度
	DIAL_ANGLE_DATA_TYPE          angle_sum_target;       //角度和目标值
	DIAL_SPEED_DATA_TYPE          speed_target;           //速度目标值
	DIAL_CURRENT_DATA_TYPE        current;                //电流目标值

}Dial_Tx_Cmd_t;

/**
 * @brief  摩擦轮命令发送结构体
 */
typedef struct{
	Fric_Work_State_e     work_state;                   //工作状态

}Fric_Tx_Cmd_t;

/**
 * @brief  摩擦轮状态检查结构体
 */
typedef struct{
	FRIC_SPEED_DATA_TYPE      speed_err[FRIC_LIST];     //速度误差
  
  uint8_t                   temp_err[FRIC_LIST];	    //温度误差
	
}Fric_Check_t;


/**
 * @brief  拨盘电机信息总结构体
 * @note   内含配置，发送，接收等结构体
 */
typedef struct{
	Dial_Config_t         config;
  Dial_Rx_Info_t        info;
	Dial_Tx_Cmd_t         cmd;

}Dial_t;

/**
 * @brief  摩擦轮电机信息总结构体
 * @note   内含配置，发送，接收等结构体
 */
typedef struct{  
	Fric_config_t         config;
	Fric_Rx_Info_t        info;
	Fric_Tx_Cmd_t         cmd;
	Fric_Check_t          check;
  
}Fric_t;


typedef struct{
	uint8_t reset_flag;
	uint8_t init_flag;
	uint8_t fire_flag;
	uint8_t firing_flag;
	uint8_t reload_flag;
	uint8_t dial_block_flag;
	uint8_t fric_block_flag;


}Shoot_Flag_t;


/**
 * @brief  发射机构总结构体
 * @note   发射机构数据接收与发送由电机分别执行
 */
typedef struct{
	Dial_t                        dial;
  Fric_t                        fric;
	Shoot_Work_State_e            shoot_work_state;
	Shoot_Mode_e                  shoot_mode; 
	Shoot_Flag_t                  shoot_flag;
	
}Shoot_t;

extern Shoot_t shoot;



/*--------------------------------对外API说明-----------------------------------*/



#endif
