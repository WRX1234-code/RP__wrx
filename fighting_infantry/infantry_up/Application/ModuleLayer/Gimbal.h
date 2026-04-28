#ifndef  __GIMBAL_H
#define  __GIMBAL_H

/*------------------------------------头文件-----------------------------------------*/
#include "motor.h"

/*------------------------------------宏定义-----------------------------------------*/

#define y_encoder_val_max    0    //pitch电机编码器最大数值
#define y_encoder_val_min    0    //pitch电机编码器最小数值

#define Y_ZERO_ANGLE         0    //yaw轴电机零点，对应车体正前方  
#define P_ZERO_ANGLE         -2.36730409    //pitch轴电机零点，对应车体正前方 
#define P_MEC_ANGLE_MAX      -2.15829611    //pitch轴电机机械限位角度最大值   
#define P_MEC_ANGLE_MIN      -2.86192465    //pitch轴电机机械限位角度最小值  

//陀螺仪模式限位，由机械限位推导
#define P_GYRO_ANGLE_MAX  (gimbal->info.imu.pitch_angle                                                             \
                           + (gimbal->misc.pitch_included_angle - (P_MEC_ANGLE_MIN - P_ZERO_ANGLE)) * 360.f / (3.1415f * 2))  \
                                                                                                                          
#define P_GYRO_ANGLE_MIN  (gimbal->info.imu.pitch_angle                                                             \
                           - ((P_MEC_ANGLE_MAX - P_ZERO_ANGLE) - gimbal->misc.pitch_included_angle) * 360.f / (3.1415f * 2))  \
                                                                                                                            
	
/*--------------------------------------结构体----------------------------------------*/	
	
/**	
  * @brief    储存处理后的陀螺仪数据的结构体	
  * @note   	看准陀螺仪方向
  */	
	typedef struct{
  	float  yaw_angle;                  
	  float  yaw_speed;                  
	  float  pitch_angle;                
	  float  pitch_speed;
	
}Imu_handle_Rx_Info_t;

/*需要删除*/
/**	
  * @brief   角度目标值增量变化系数配置结构体，用于调整增量的变化程度	
  * @note    自行配置，类似灵敏度
  */	
typedef struct{
	//遥控器灵敏度
	float  rc_yaw_mec_k;      
	float  rc_yaw_gyro_k;     
	float  rc_pitch_mec_k;    
	float  rc_pitch_gyro_k;   

	//键鼠灵敏度
	float  key_yaw_mec_k;     
	float  key_yaw_gyro_k;    
	float  key_pitch_mec_k;   
	float  key_pitch_gyro_k;  
	 /*需要保存*/
	float  yaw_bias_add;      //零偏角度

}Cal_Cfg_Rx_Info_t;


///**	
//  * @brief   视觉信息实时接收结构体
//  */	
//typedef struct{
//	
//	
//	
//}Vision_Rt_Rx_Info_t;


///**	
//  * @brief     裁判系统数据包接收结构体
//  */	
//typedef struct{
//	            
//	            
//	            

//}Judge_Rx_Pkt_t;


/**	
  * @brief      云台数据接收总结构体
  */	
typedef struct{
	Imu_handle_Rx_Info_t    imu;
  Cal_Cfg_Rx_Info_t       cal;
//  Vision_Rt_Rx_Info_t     vision;
//	Judge_Rx_Pkt_t          judge;
	
}Gimbal_Rx_Info_t;


/**	
  * @brief     pitch轴电机命令发送结构体
  * @note 
  */	
typedef struct{
	float  output;                  //输出，类比电流，扭矩
	float  mec_angle_target;
	float  gyro_angle_target;
	

}Pitch_Tx_Cmd_t;


/**	
  * @brief      yaw轴电机命令发送结构体                
  * @note       用于更新板间通信
  */	
typedef struct{
	float  output;
	float  mec_angle_target;
	float  gyro_angle_target;

}Yaw_Tx_Cmd_t;


///**	
//  * @brief   视觉命令发送结构体
//  * @note    用于暂存视觉所需命令，等待发送
//  */	
//typedef struct{



//}Vision_Tx_Cmd_t;


/**	
  * @brief      云台数据发送总结构体
  */	
typedef struct{
	Pitch_Tx_Cmd_t     pitch;
  Yaw_Tx_Cmd_t       yaw;
//  Vision_Tx_Cmd_t    vision;
}Gimbal_Tx_Cmd_t;


/**	
  * @brief    云台内置标志位结构体
  * @note     私有化，不要动
  */	
typedef struct{
	uint8_t init_zero_flag;      //初始化车头回正标志位，完成置 1，未完成置 0
	
	uint8_t zero_bias_flag;      //零偏校准标志位，完成置 1，未完成置 0


}Gimbal_inn_Flag_t;


/**	
  * @brief     云台数据杂项结构体
  * @note      储存的都是便于检查云台情况的数据，可能只更新一次
  */	
typedef struct{
	float x_offset;
	float y_offset;
	float z_offset;
	
	float yaw_included_angle;         //yaw轴相对零点角度
	float pitch_included_angle;       //pitch轴相对零点角度

}Gimbal_Misc_t;


/**	
  * @brief      云台总结构体
  */	
typedef struct{
	Motor_DM_t*        pitch;
  Gimbal_Rx_Info_t   info;
  Gimbal_Tx_Cmd_t    cmd;
	Gimbal_inn_Flag_t  flag;
	Gimbal_Misc_t      misc;
	
}Gimbal_t;

extern Gimbal_t gimbal;

/*-------------------------------对内API说明----------------------------------*/

static void Gimbal_Imu_data_Update(Gimbal_t* gimbal);
static void Gyro_bias_manage(Gimbal_t* gimbal);

/*-------------------------------对外API说明----------------------------------*/

void Gyro_zero_bias(Gimbal_t* gimbal);
void Gimbal_Mec_Update(Gimbal_t* gimbal);
void Gimbal_Gyro_Update(Gimbal_t* gimbal);
void Gimbal_Self_Aim_Update(Gimbal_t* gimbal);
void Gimbal_Send(Gimbal_t* gimbal);
void Gimbal_PID_Cal(Gimbal_t* gimbal);
void Gimbal_Sleep(Gimbal_t* gimbal);
void Gimbal_Work(Gimbal_t* gimbal);


#endif
