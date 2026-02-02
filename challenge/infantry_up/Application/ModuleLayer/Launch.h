#ifndef  __LAUNCH_H
#define  __LAUNCH_H

/*--------------------------------头文件-----------------------------------*/

#include "stdint.h"
#include "shoot_Base.h"
#include "motor.h"

/*--------------------------------宏定义-----------------------------------*/

#define  FRIC_NUM                                3                        //摩擦轮数量，有六摩 6，三摩 3，二摩 2 
#define  FRIC_SPEED_DATA_DIRECTION_MENAGE        k = 1                    //k用于矫正摩擦轮转向     
 
#define  IS_CHECK_DRIC_TEMP                       0                       //是否检查摩擦轮温度,是为 1，不是为 0

#define  MUZZLE_HEAT_MAX                          200/*需要修改*/          //裁判系统默认枪口最大温度，超过吃罚

#define  FRIC_SPEED_DATA_TYPE                  int16_t                    //摩擦轮速度数据类型  
#define  FRIC_CURRENT_DATA_TYPE                int16_t                    //摩擦轮电流数据类型

/*--------------------------------枚举-------------------------------------*/

//摩擦轮数量枚举

#if  FRIC_NUM == 6
typedef enum{
  FRICTION_B_UP = 0,                    
	FRICTION_B_L,                         
	FRICTION_B_R,                         
	
  FRICTION_F_UP,                        
	FRICTION_F_L,                            
	FRICTION_F_R,                            
        
  FRICTION_LIST,
}Fric_Zone_e; 

#elif FRIC_NUM == 3
typedef enum{
  FRICTION_UP = 0,                      
  FRICTION_R,                           
  FRICTION_L,                           
  
	FRICTION_LIST,
}Fric_Zone_e;

#elif FRIC_NUM ==2
typedef enum{
  FRICTION_R = 0,                       
  FRICTION_L,                           

  FRICTION_LIST,
}Fric_Zone_e;

#endif

/*---------------------------------结构体------------------------------------*/



/**	
  * @brief    摩擦轮数据实时接收结构体
  */	
typedef struct{ 
	FRIC_SPEED_DATA_TYPE                 speed;             
  FRIC_CURRENT_DATA_TYPE               current;           
	uint8_t                              temperature;       //部分电机没有反馈

}Fric_Rt_Rx_Info_t;


/**	
  * @brief    摩擦轮基本参数配置结构体
  */	
typedef struct{
  FRIC_SPEED_DATA_TYPE          normal_speed_target;        //正常速度目标值
	FRIC_SPEED_DATA_TYPE          high_temp_speed_target;     //高温下速度目标值，用于冷却，小于normal_speed_target
	FRIC_SPEED_DATA_TYPE          up_speed_target;
	uint8_t                       temp_max;                   //温度最大值，超过触发高温
  FRIC_SPEED_DATA_TYPE          speed_err_max;              //速度误差最大值
  uint8_t                       temp_err_max;	              //温度误差最大值

}Fric_Base_Cfg_Rx_Info_t;

/**	
  * @brief    摩擦轮堵转处理参数配置结构体
  */	
typedef struct{
	FRIC_SPEED_DATA_TYPE           speed_max;                 //速度阈值
	FRIC_CURRENT_DATA_TYPE         current_min;               //电流阈值
	uint8_t                        block_time_max;            //堵转时间阈值                
  uint8_t                        menage_time_max;           //处理时间阈值                   
  uint8_t                        block_time;                //堵转时间         
	
}Fric_Block_Cfg_Rx_Info_t;


/**	
  * @brief    摩擦轮弹速自适应参数配置结构体
  */	
typedef struct{
	float                           speed_max;                     //合理弹速最大值
	float                           speed_min;                     //合理弹速最小值
	uint8_t                         first_bullet_shield_time;      //第一发弹屏蔽时间
	float                           ideal_speed_max;               //理想弹速最大值
	float                           ideal_speed_min;               //理想弹速最小值
	float                           ideal_death_value;             //理想弹速区间死区范围
	float                           overspeed_adjust_speed;        //超速时用于调整的速度
	float                           high_adjust_speed;             //高速时的调整速度
	float                           low_adjust_speed;              //低速时的调整速度
	
	float                           high_adjust_value;             //高速时的速度调整比例
	float                           low_adjust_value;              //低速时的速度调整比例
	
	uint8_t                         more_cnt_max;                  //高速次数阈值
	uint8_t                         less_cnt_max;                  //低速次数阈值
 
}Fric_Self_Adapt_Cfg_Rx_Info_t ;


/**	
  * @brief    摩擦轮参数配置输入总结构体
  */	
typedef struct{
	Fric_Base_Cfg_Rx_Info_t                  base_cfg;        
	Fric_Block_Cfg_Rx_Info_t                 block_cfg[FRICTION_LIST];       
	Fric_Self_Adapt_Cfg_Rx_Info_t             adapt_cfg;       
	
}Fric_Cfg_Rx_Info_t;


/**	
  * @brief    摩擦轮数据输入总结构体
  */	
typedef struct{
	Fric_Cfg_Rx_Info_t         cfg_rx_info;
  Fric_Rt_Rx_Info_t          rt_rx_info[FRICTION_LIST];

}Fric_Rx_Info_t;


/**	
  * @brief    发射机构数据输入总结构体
  */	
typedef struct{
	Fric_Rx_Info_t        fric_info;
 
}Launch_Rx_Info_t;


/**	
  * @brief    摩擦轮目标值结构体
  */	
typedef struct{
  FRIC_CURRENT_DATA_TYPE     output;                      
	FRIC_SPEED_DATA_TYPE       speed_target;
	
}Fric_Tar_t;


/**	
  * @brief    摩擦轮数据检查结构体
  * @note     便于查看情况
  */	
typedef struct{
	FRIC_SPEED_DATA_TYPE      speed_target;
	FRIC_SPEED_DATA_TYPE      speed_err[FRICTION_LIST];    
  int8_t                   temp_err[FRICTION_LIST];	   
	
}Fric_Check_t;


/**	
  * @brief    摩擦轮组装总结构体
  */	
typedef struct{  
	Motor_RM_Group_t*     group;
	Fric_Tar_t            tar;
	Fric_Check_t          check;
  
}Fric_Assembly_t;


/**	
  * @brief    发射机构内部标志位结构体
  * @note    文件私有，不要调用
  */	
typedef struct{
  uint8_t fric_block_flag;                //摩擦轮堵转标志位，堵转置 1，不堵转置 0
	uint8_t fric_normal_speed_flag;         //摩擦轮转速正常标志位，正常置 1，不正常置 0
	uint8_t fric_high_temp_flag;            //摩擦轮高温标志位，高温置 1，否则置 0
}Launch_Inn_Flag_t;


/**	
  * @brief    裁判系统数据包结构体
  */	
typedef struct{
	float now_speed;                 //当前弹速           
	float shoot_freq;                //射频         
	float muzzle_heat;               //枪口温度        

}Judge_Rx_Pkt_t;


/**	
  * @brief    发射机构数据杂项结构体
  * @note    可自行添加变量便于查看
  */	
typedef struct{
	uint16_t safe_cnt;             //记录开关发射机构的总次数
	
}Launch_Misc_t;


/**	
  * @brief    发射机构总结构体
  */	
typedef struct{
	Fric_Assembly_t     assembly;
	Shoot_t*            base;
	Launch_Rx_Info_t    info;
	Judge_Rx_Pkt_t      judge;
  Launch_Inn_Flag_t   flag;
  Launch_Misc_t       misc;

}Launch_t;

extern Launch_t launch;


void Launch_Data_Update(Launch_t* launch);
void Launch_Flag_Update(Launch_t* launch);
uint8_t Fric_Block_Check(Launch_t* launch);
void Fric_State_Check(Launch_t* launch);
void Launch_Speed_Self_Adapt(Launch_t* launch);
void Fric_Pid_Cal(Launch_t* launch);
void Launch_Work(Launch_t* launch);

#endif
