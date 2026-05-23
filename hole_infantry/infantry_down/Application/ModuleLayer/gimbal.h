#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "stdint.h"
#include <stdbool.h>

#define   YAW_MEC_ZERO_ANGLE          1.58
#define   PITCH_MEC_ZERO_ANGLE        1.58
#define   PITCH_MEC_MAX_ANGLE         1.58
#define   PITCH_MEC_MIN_ANGLE         1.58

#define   PITCH_IMU_MAX_ANGLE         1
#define   PITCH_IMU_MIN_ANGLE         1

#define   PI      3.1415926

typedef enum{
	G_SLEEP,
	G_INIT,
	G_BOSS,
	G_SLAVE,

}Gimbal_Mode_e;


typedef enum{
	FRONT,
	RIGHT,
	BEHIND,
	LEFT,
	DIRECT_CNT,

}Gimbal_Direct_e;

typedef struct{
	float yaw_mec_tar;
	float pitch_mec_tar;
	float yaw_imu_tar;
	float pitch_imu_tar;
	
	float gimbal_height;


}Gimbal_Target_t;


typedef struct{
  float yaw_mec;
	float pitch_mec;
	float yaw_imu;
	float pitch_imu;
	
	float yaw_mec_err;
	
	float gimbal_height;

	

}Gimbal_Info_t;

typedef struct{
	float rc_yaw_mec_step;
	float rc_pitch_mec_step;
	float rc_yaw_imu_step;
	float rc_pitch_imu_step;
	float key_yaw_mec_step;
	float key_pitch_mec_step;
	float key_yaw_imu_step;
	float key_pitch_imu_step;
	
  float yaw_zero[DIRECT_CNT];
	
}Gimbal_Config_t;


typedef struct Gimbal_Struct_t{
  Gimbal_Mode_e      mode;
  Gimbal_Target_t    target;
	Gimbal_Info_t      info;
	Gimbal_Config_t    config;
  
	bool        gimbal_reset_flag;
	
	
	void (*work)(struct Gimbal_Struct_t* gimbal);
	
	
}Gimbal_t;

extern  Gimbal_t   gimbal;


#endif

