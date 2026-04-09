/*---------------------------------------------------------------------------------
 *  @file       shoot_base.h
 *  @brief      发射机构基础模块对外接口
 *  @author     WRX
 *  @date       2025.11.25
 *  @version    2.1.1
 *-------------------------------------------------------------------------------*/

#ifndef __SHOOT_BASE_H
#define __SHOOT_BASE_H


/**-------------------------------------------------------------------------------
 *                               机械坐标系约定
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


/**-------------------------------------------------------------------------------
 *                                  如何使用
 * -------------------------------------------------------------------------------
 * (#)在shoot_base_h文件中的宏定义可配置区按照注释提示配置宏定义
 *
 * (#)在shoot_base_c文件中的Shoot_Init函数中配置可配置参数
 *
 * (#)在外部需要有另一个文件，执行发射机构的高级功能，如PID计算，摩擦轮堵转，温度，速度检测，弹速自适应等
 *
 * (#)外部文件中需要更新Shoot_Rt_Rx_Info_t中的参数，标志位，以及Vision_Tx_Cmd_t中的is_ready_flag
 *
 * (#)Shoot_Tx_Cmd_t中的结构体分别是拨盘，摩擦轮的基础输出，以及视觉所需要的输出
 *
 * (#)Shoot_Inner_Flag_t中的标志位是本文件私有，不要在外部修改
 *
 * (#)在外部文件中调用Shoot_Base_Work函数即可
 * ------------------------------------------------------------------------------*/            

/*--------------------------------外部头文件引用---------------------------------*/
#include "stm32f4xx.h"
#include <stdint.h>
#include "math.h"

/*---------------------------宏定义只读区 1（禁止修改）----------------------------*/ 

//为拨盘角度数据类型匹配数字
#define  TYPE_UINT16                  0            //匹配 uint16_t
#define  TYPE_UINT32                  1            //匹配uint32_t
#define  TYPE_FLOAT                   2            //匹配 float

#define  TYPE_ANGLE                   TYPE_FLOAT

//根据拨盘电机角度对应数字来定义角度差数据类型和绝对值宏定义
#if  TYPE_ANGLE == TYPE_UINT16 
#define  DIAL_ANGLE_ERR_DATA_TYPE         int16_t
//取绝对值
#define  abs_cal(x)                 abs((DIAL_ANGLE_ERR_DATA_TYPE)(x))
#define  err_abs_cal(x,y)           abs((DIAL_ANGLE_ERR_DATA_TYPE)(x) - (DIAL_ANGLE_ERR_DATA_TYPE)(y))

#elif  TYPE_ANGLE == TYPE_UINT32
#define  DIAL_ANGLE_ERR_DATA_TYPE         int32_t
#define  abs_cal(x)                 abs((DIAL_ANGLE_ERR_DATA_TYPE)(x))
#define  err_abs_cal(x,y)           abs((DIAL_ANGLE_ERR_DATA_TYPE)(x )- (DIAL_ANGLE_ERR_DATA_TYPE)y)
 
#elif  TYPE_ANGLE == TYPE_FLOAT
#define  DIAL_ANGLE_ERR_DATA_TYPE         float
#define  abs_cal(x)                 fabs((DIAL_ANGLE_ERR_DATA_TYPE)(x))
#define  err_abs_cal(x,y)           fabs((DIAL_ANGLE_ERR_DATA_TYPE)(x) - (DIAL_ANGLE_ERR_DATA_TYPE)(y))

#else
#define  DIAL_ANGLE_ERR_DATA_TYPE         void
#error   " TYPE_ANGLE 未正确配置! "

#endif


/*--------------------------------宏定义可配置区---------------------------------*/

#define  ANGLE_ERR_TYPE        			 float

//拨盘
#define  DIAL_MOTOR_TYPE                  M_2006                  //拨盘电机类型，从Dial_Motor_Type_e里面选

#define  DIAL_MEC_LIMIT                   1                        //拨盘有无机械限位，无为 -1，有为 1

#define  DIAL_IS_ABSOLUTE_ANGLE           1                        //拨盘是否有绝对角度，有为 1，没有为 0

#define  DIAL_PUSHER_NUM                  8                        //拨盘拨爪数量

#define  DIAL_ANGLE_MAX                   32768                    //拨盘机械角度数值最大值，如相对角度有8191，绝对角度有10.f
#define  DIAL_ANGLE_MIN                   0                        //拨盘机械角度数值最小值，如 相对角度的0，绝对角度有的是-10.f

#define  DIAL_ANGLE_DATA_TYPE             int16_t              //拨盘角度数据类型
#define  DIAL_SPEED_DATA_TYPE             int16_t                  //拨盘速度数据类型
#define  DIAL_CURRENT_DATA_TYPE           int16_t                  //拨盘电流数据类型

#define  DIAL_ANGLE_SUM_DATA_TYPE         int32_t                  //拨盘角度和数据类型


//摩擦轮
#define  FRIC_NUM                         3                        //摩擦轮数量，分六摩 6，三摩 3，二摩 2

#define  FRIC_SPEED_DATA_TYPE             int16_t                  //摩擦轮速度数据类型
#define  FRIC_CURRENT_DATA_TYPE           int16_t                  //摩擦轮电流数据类型


/*---------------------------宏定义只读区 2（禁止修改）----------------------------*/ 

#define  RELATIVE_ANGLE_STOP     (DIAL_IS_ABSOLUTE_ANGLE == 0 && err_abs_cal(shoot->cmd.dial_tx_cmd.angle_sum_target , shoot->misc.angle_sum) \
			                      <= shoot->info.cfg_rx_info.base_cfg_info.stop_angle_err_max )  \

#define  ABSOLUTE_ANGLE_STOP     (DIAL_IS_ABSOLUTE_ANGLE == 1 && abs_cal(Half_Cir_Handle(shoot->cmd.dial_tx_cmd.angle_target - shoot->info.rt_rx_info.dial_info.angle)) \
		                         <= shoot->info.cfg_rx_info.base_cfg_info.stop_angle_err_max)  \
				
																					 
/*----------------------------------枚举定义-------------------------------------*/

/**
 * @brief  拨盘电机类型枚举
 */
typedef enum{
	GM_6020,
	M_3508,
	M_2006,
	DM_J4310,
	KT_4005,
	
}Dial_Motor_type_e;


/**
 * @brief  拨盘运动状态枚举
 */
typedef enum{
	SLEEP = 0,                            //睡眠状态
	RESETING,                             //复位状态
	WAITING,                              //等待状态
	RELOAD,                               //补弹状态
	RECOIL,                               //堵转处理状态

}Dial_Work_State_e;

/**
 * @brief  摩擦轮运动状态枚举
 */
typedef enum{
	RUN = 0,                               //转动
	STOP,                                  //不转
	
}Fric_Work_State_e;

/**
 * @brief  拨盘转动模式枚举
 */
typedef enum{
	DIAL_SPEED = 0,                       //速度环
	DIAL_ANGLE,                           //角度环

}Dial_Mode_e;

/**
* @brief  发射机构开火模式枚举
 */
typedef enum{
	CEASEFIRE = 0,                          //停火模式
	SINGLE_SHOT,                            //单发模式
	REPEAT_SHOT,                            //连发模式
  
}Shoot_Mode_e;

/**
* @brief  发射机构工作状态枚举
 */
typedef enum{
	LOCKED = 0,                             //关保险状态，无法射击
	INITING,                                //初始化状态，无法射击
	UNLOCK,                                 //开保险状态，允许射击

}Shoot_Work_State_e;


/**
* @brief  拨盘速度环停下后状态枚举
 */
typedef enum{
	STAND,                                   //原地定住
	FORWARD,                                 //向前补位
	BACKWARD,                                //向后补位
	ROUNDING_UP,                             //四舍五入向前或后补位

}Dial_Speed_Stop_Mode_e;




/*--------------------------------结构体/联合体----------------------------------*/

/**
 * @brief  拨盘实时数据接收结构体
 * @note  数据在外部文件更新
 */
typedef struct{
	DIAL_ANGLE_DATA_TYPE                 angle;           //角度
	DIAL_SPEED_DATA_TYPE                 speed;           //速度
  DIAL_CURRENT_DATA_TYPE               current;         //电流

}Dial_Rt_Rx_Info_t;


/**
* @brief  发射机构实时标志位接收结构体
 * @note  标志位在外部文件更新，内部处理
 */
typedef struct{
	uint8_t is_sleep_flag;                  //发射机构睡眠/卸力标志位，开控置0，关控置1，若置1后需重新复位，机器人阵亡时无需复位不应置1  
	uint8_t is_mtr_offline_flag; 			//发射机构是否有相关电机掉线，1为掉线，0为在线，掉线了不响应开火操作 && is_ready_flag=0，但不一定进入sleep状态
	uint8_t fire_mode_flag;                 //开火模式标志位，单发为 0，连发为 1 
	uint8_t elec_level_flag;                //电平标志位，高电平为 1，低电平为 0
    uint8_t init_flag;                      //初始化标志位，置0时进入复位，复位完成后置1，用于外部手动复位
	uint8_t run_limit_flag;					//运行限制标志位，在功率限制，摩擦轮堵转等情况下，在不上锁发射机构的前提下停止运行
}Flag_Rt_Rx_Info_t;

/**
* @brief  发射机构实时接收总结构体
 * @note  数据在外部文件更新
 */
typedef struct{
	Dial_Rt_Rx_Info_t                dial_info;
 
	Flag_Rt_Rx_Info_t                flag_Info;
	
}Shoot_Rt_Rx_Info_t;


/**
 * @brief  拨盘数据基本配置输入结构体
 */
typedef struct{
	//复位配置
	
	//相对角度专用
	DIAL_SPEED_DATA_TYPE          reset_speed;                   //有相对角度的拨盘电机复位速度，低速
	DIAL_ANGLE_DATA_TYPE          reset_adjust_angle;            //拨盘复位后零点偏置角度，用于调整弹丸在弹链中的位置
	uint16_t                      reset_speed_work_time_max;     //速度环复位最大工作时间
	//绝对角度专用
	DIAL_ANGLE_DATA_TYPE          reset_offist_angle;            //有绝对角度的拨盘电机复位时的偏置角度，用于确定角度换数值周期
	//相对，绝对角度公用
	uint16_t                      reset_angle_work_time_max;     //角度环复位最大工作时间
	
	//补弹配置
	DIAL_ANGLE_DATA_TYPE          oneshot_angle;                 //拨一颗弹，拨盘电机转过的角度
	DIAL_SPEED_DATA_TYPE          reload_speed;                  //补弹速度，较高速
  Dial_Mode_e                   repeat_shot_mode;              //拨盘连发转动模式，分速度环和角度环
	uint16_t                       repeat_shot_period;            //拨盘角度环连发周期
  uint16_t                      state_work_time_max;           //拨盘角度环补弹退弹最大工作时间
	Dial_Speed_Stop_Mode_e        speed_stop_mode;               //拨盘连发停止的归位模式
	
	//其余配置
	DIAL_ANGLE_DATA_TYPE          stop_angle_err_max;            //判断停止角度误差最大值
	
}Dial_Base_Cfg_Rx_Info_t;


/**
 * @brief  拨盘堵转状态数据配置输入结构体
 */
typedef struct{
	//第一种堵转判断，依靠速度，电流，时间
	DIAL_SPEED_DATA_TYPE           speed_max;                          //堵转判断最大速度
	DIAL_CURRENT_DATA_TYPE         current_min;                        //堵转判断最小电流
	uint16_t block_time_max;                                            //堵转判断最大时间
  uint16_t block_time;                                                //堵转时间
	
	//第二种堵转判断，依靠角度和误差积分值，大于阈值就堵转，单发完成一次清零，角度环连发完成一次清零
	//相对角度专用
	DIAL_ANGLE_SUM_DATA_TYPE         angle_sum_err_integral;           //角度和误差
	DIAL_ANGLE_SUM_DATA_TYPE         angle_sum_err_integral_max;       //角度和误差积分最大值
	//绝对角度专用
	DIAL_ANGLE_DATA_TYPE             angle_err_integral;               //角度误差
	DIAL_ANGLE_DATA_TYPE             angle_err_integral_max;           //角度误差积分最大值
	//相对，绝对角度共用
	float                            integral_value;                   //积分系数，用来对角度或角度和误差积分
	uint8_t                          block_judge_type;                 //堵转判断类型，第一种为 0，第二种为 1
	
}Dial_Block_Cfg_Rx_Info_t;


/**
* @brief  发射机构配置输入总结构体
 */
typedef struct{
	Dial_Base_Cfg_Rx_Info_t          base_cfg_info;                     //基本配置
	Dial_Block_Cfg_Rx_Info_t         reset_speed_block_cfg_info;        //速度环复位堵转配置
  Dial_Block_Cfg_Rx_Info_t         speed_block_cfg_info;              //速度环堵转配置
  Dial_Block_Cfg_Rx_Info_t         angle_block_cfg_info;              //角度环堵转配置
	
}Shoot_Cfg_Rx_Info_t;



/**
* @brief  拨盘命令发送结构体
* @note   只发送拨盘状态，模式，目标值，以及视觉需要的拨弹进度
 */
typedef struct{
	Dial_Work_State_e             work_state;             //工作状态
	Dial_Mode_e                   mode;                   //工作模式
  
	//相对角度专用
	DIAL_ANGLE_SUM_DATA_TYPE      angle_sum_target;       //角度和目标值
	//绝对角度专用
	DIAL_ANGLE_DATA_TYPE          angle_target;           //角度目标值
	//角度公用
	DIAL_SPEED_DATA_TYPE          speed_target;           //速度目标值
	DIAL_CURRENT_DATA_TYPE        current_target;         //电流目标值

}Dial_Tx_Cmd_t;

/**
* @brief  摩擦轮命令发送结构体
* @note   只发送摩擦轮状态
 */
typedef struct{
	Fric_Work_State_e          work_state;                  //工作状态

}Fric_Tx_Cmd_t;

/**
 * @brief  视觉命令发送结构体
 */
typedef struct{
	float                         reload_sche;              //拨弹进度
	uint8_t                       is_ready_flag;            //能否立即打弹标志位，不能为 0，能为 1,提供给视觉判断发弹时机
}Vision_Tx_Cmd_t;


/**
* @brief  发射机构杂项数据结构体
* @note  用于储存不属于输入输出，标志位，状态,裁判系统等的数据，不可或缺，放在此处便于debug 
 */
typedef struct{
	DIAL_ANGLE_SUM_DATA_TYPE      angle_sum_start;        //起始角度和，记录复位调整角度后的角度和
	DIAL_ANGLE_SUM_DATA_TYPE      angle_sum;              //角度和
  DIAL_ANGLE_DATA_TYPE          beyond_angle;           //拨盘当前角度超出后面角度环集合目标值的角度
	DIAL_ANGLE_ERR_DATA_TYPE      angle_err;              //拨盘角度差
	//绝对角度专用
	DIAL_ANGLE_DATA_TYPE          absolute_angle_target[DIAL_PUSHER_NUM];    //拨盘绝对角度下的角度目标值
	DIAL_ANGLE_DATA_TYPE          front_absolute_angle_target;               //上一个绝对角度目标值
	DIAL_ANGLE_DATA_TYPE          behind_absolute_angle_target;              //下一个绝对角度目标值
	
}Shoot_Misc_t;

/**
* @brief  发射机构内部标志位结构体
* @note   内含的标志位均属发射机构私有，内部自行更新，内部处理
 */
typedef struct{
	
	uint8_t dial_block_flag;              //拨盘非正常堵转标志位，堵转置 1，未堵转置 0
	//绝对相对角度专用
	uint8_t reset_speed_flag;             //复位时速度环完成标志位，完成置 1，未完成置 0
	
}Shoot_Inner_Flag_t;


/**
* @brief  发射机构输入总结构体
 */
typedef struct{
	Shoot_Rt_Rx_Info_t         rt_rx_info;
  Shoot_Cfg_Rx_Info_t        cfg_rx_info;

}Shoot_Rx_Info_t;
/**
* @brief  发射机构命令发送总结构体
 */
typedef struct{
	Dial_Tx_Cmd_t                      dial_tx_cmd;
	Fric_Tx_Cmd_t                      fric_tx_cmd;
	Vision_Tx_Cmd_t                    vision_tx_cmd;
	
}Shoot_Tx_Cmd_t;

/**
 * @brief  发射机构总结构体
 * @note   发射机构数据接收与发送由电机分别执行
 */
typedef struct{
	Shoot_Rx_Info_t               info;
	Shoot_Tx_Cmd_t                cmd;
	Shoot_Work_State_e            work_state;
	Shoot_Mode_e                  mode; 
	Shoot_Inner_Flag_t            flag;
	Shoot_Misc_t                  misc;
	
}Shoot_t;

extern Shoot_t shoot;

/*--------------------------------对内API说明-----------------------------------*/

static DIAL_ANGLE_ERR_DATA_TYPE Half_Cir_Handle(DIAL_ANGLE_ERR_DATA_TYPE err);
static void Angle_Sum_Calculate(Shoot_t* shoot);
static DIAL_ANGLE_DATA_TYPE Absolute_Angle_Wrap(DIAL_ANGLE_DATA_TYPE unwraped_angle);
static void Angle_Target_Switch(Shoot_t* shoot);
static void Absolute_Angle_Target_Init(Shoot_t* shoot);
static void Absolute_Angle_Target_Transfor(Shoot_t* shoot);


/*--------------------------------对外API说明-----------------------------------*/

void Shoot_Init(Shoot_t* shoot);
void Shoot_Work_State_Update(Shoot_t* shoot);
void Shoot_Mode_Update(Shoot_t* shoot);
uint8_t Dial_Block_Check(Dial_Rt_Rx_Info_t* rt_info,Shoot_Misc_t* misc,Dial_Block_Cfg_Rx_Info_t* cfg_info,Dial_Tx_Cmd_t* cmd);
void Dial_Work_State_Update(Shoot_t* shoot);
void Shoot_Sleep(Shoot_t* shoot);
void Shoot_Base_Work(Shoot_t* shoot);


#endif
