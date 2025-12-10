#ifndef  __GIMBAL_H
#define  __GIMBAL_H

#include "motor.h"

#define y_encoder_val_max    0    //
#define y_encoder_val_min    0    //

#define Y_ZERO_ANGLE         0    //  
#define P_ZERO_ANGLE         0    //   
#define P_MEC_ANGLE_MAX      0    //   
#define P_MEC_ANGLE_MIN      0    // 

#define P_GYRO_ANGLE_MAX  (gimbal->info.imu.pitch_angle                                                             \
                           + ((P_MEC_ANGLE_MAX - P_ZERO_ANGLE) - gimbal->misc.pitch_included_angle) * 360.f / 2.f)  \
                                                                                                                          
#define P_GYRO_ANGLE_MIN  (gimbal->info.imu.pitch_angle                                                             \
                           - (gimbal->misc.pitch_included_angle - (P_MEC_ANGLE_MIN - P_ZERO_ANGLE)) * 360.f / 2.f)  \
                                                                                                                            


typedef struct{
	float  yaw_angle;
	float  yaw_speed;
	float  pitch_angle;
	float  pitch_speed;
	
}Imu_handle_Rx_Info_t;

typedef struct{
	float  rc_yaw_mec_k;      //
	float  rc_yaw_gyro_k;     // 
	float  rc_pitch_mec_k;    //
	float  rc_pitch_gyro_k;   //
	
	float  key_yaw_mec_k;     //
	float  key_yaw_gyro_k;    //
	float  key_pitch_mec_k;   // 
	float  key_pitch_gyro_k;  //
	 
	float  yaw_bias_add;      //

}Cal_Cfg_Rx_Info_t;


typedef struct{


}Vision_Rt_Rx_Info_t;


typedef struct{
	            
	            
	            

}Judge_Rx_Pkt_t;


typedef struct{
	Imu_handle_Rx_Info_t    imu;
  Cal_Cfg_Rx_Info_t       cal;
  Vision_Rt_Rx_Info_t     vision;
	Judge_Rx_Pkt_t          judge;
	
}Gimbal_Rx_Info_t;


typedef struct{
	float  output;
	float  mec_angle_target;
	float  gyro_angle_target;
	

}Pitch_Tx_Cmd_t;

typedef struct{
	float  output;
	float  mec_angle_target;
	float  gyro_angle_target;

}Yaw_Tx_Cmd_t;

typedef struct{



}Vision_Tx_Cmd_t;


typedef struct{
	Pitch_Tx_Cmd_t     pitch;
  Yaw_Tx_Cmd_t       yaw;
  Vision_Tx_Cmd_t    vision;
}Gimbal_Tx_Cmd_t;


typedef struct{
	uint8_t init_zero_flag;
	
	uint8_t zero_bias_flag;


}Gimbal_inn_Flag_t;





typedef struct{
	float x_offset;
	float y_offset;
	float z_offset;
	
	float yaw_included_angle;
	float pitch_included_angle;

}Gimbal_Misc_t;


typedef struct{
	Motor_DM_t*        pitch;
  Gimbal_Rx_Info_t   info;
  Gimbal_Tx_Cmd_t    cmd;
	Gimbal_inn_Flag_t  flag;
	Gimbal_Misc_t      misc;
	
}Gimbal_t;


static void Gimbal_Imu_data_Update(Gimbal_t* gimbal);
static void Gyro_zero_bias(Gimbal_t* gimbal);
static void Gyro_bias_manage(Gimbal_t* gimbal);
void Gimbal_Mec_Update(Gimbal_t* gimbal);
void Gimbal_Gyro_Update(Gimbal_t* gimbal);
void Gimbal_Send(Gimbal_t* gimbal);
void Gimbal_PID_Cal(Gimbal_t* gimbal);
void Gimbal_Sleep(Gimbal_t* gimbal);
void Gimbal_Work(Gimbal_t* gimbal);


#endif
