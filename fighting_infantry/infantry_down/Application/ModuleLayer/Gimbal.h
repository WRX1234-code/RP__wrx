#ifndef  __GIMBAL_H
#define  __GIMBAL_H

#include "DM_motor.h"

#define y_encoder_val_max    0    //pitch电机编码器最大数值
#define y_encoder_val_min    0    //pitch电机编码器最小数值

#define Y_ZERO_ANGLE         1.52158809    //yaw轴电机零点，对应车体正前方  
#define P_ZERO_ANGLE         -2.371522679    //pitch轴电机零点，对应车体正前方 
#define P_MEC_ANGLE_MAX      -2.00518322    //pitch轴电机机械限位角度最大值   
#define P_MEC_ANGLE_MIN      -2.85722661    //pitch轴电机机械限位角度最小值  


//陀螺仪模式限位，由机械限位推导
#define P_GYRO_ANGLE_MAX  (gimbal->info.rt_info.pitch_imu                                                             \
                           + (gimbal->misc.pitch_included_angle - (P_MEC_ANGLE_MIN - P_ZERO_ANGLE)) * 360.f / (3.1415f * 2))  \
                                                                                                                          
#define P_GYRO_ANGLE_MIN  (gimbal->info.rt_info.pitch_imu                                                             \
                           - ((P_MEC_ANGLE_MAX - P_ZERO_ANGLE) - gimbal->misc.pitch_included_angle) * 360.f / (3.1415f * 2))  \

typedef struct{
	float pitch_imu;
	float yaw_imu;
	float yaw_v;
	float pitch_v;
	float pitch_mec;
	
	float vision_pitch_tar;     //自瞄pitch目标值
	float vision_yaw_tar;       //自瞄yaw目标值
	
}Gimbal_Rt_Rx_Info_t;

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
	
	float head_to[8];
	 
}Gimbal_Cfg_Rx_Info_t;


typedef struct{
	Gimbal_Rt_Rx_Info_t      rt_info;
Gimbal_Cfg_Rx_Info_t       cfg_info;

}Gimbal_Rx_Info_t;
	

typedef struct{
	float pitch_imu_tar;        //pitch陀螺仪模式目标角度
	float yaw_imu_tar;
	float pitch_mec_tar;        //pitch机械模式目标角度
	float yaw_mec_tar;
	uint8_t yaw_offset;         //ywa轴发射后角度偏移

}Gimbal_Tx_Cmd_t;

typedef struct{
	float pitch_included_angle; 
  float yaw_included_angle;

}Gimbal_Misc_t;
	
	


typedef struct{
  Motor_DM_t*       yaw;
	Gimbal_Rx_Info_t  info;
  Gimbal_Tx_Cmd_t   cmd;
  Gimbal_Misc_t     misc;
	
}Gimbal_t;

extern Gimbal_t gimbal;


void Gimbal_Board_Update(Gimbal_t* gimbal);
void Gimbal_Reset_Init(Gimbal_t* gimbal);
void Gimbal_Mec_Update(Gimbal_t* gimbal);
void Gimbal_Gyro_Update(Gimbal_t* gimbal);
void Vision_Self_Aim_Update(Gimbal_t* gimbal);
void Gimbal_Pid_Cal(Gimbal_t* gimbal);
void Gimbal_Send(Gimbal_t* gimbal);
void Gimbal_Work(Gimbal_t* gimbal);


#endif
