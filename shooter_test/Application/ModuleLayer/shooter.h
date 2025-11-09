#ifndef __SHOOTER_H
#define __SHOOTER_H

#include "pid.h"
#include "rm_motor.h"

#define  FRIC_SPEED_TARGET  2000.f    //5000.f
#define  DIAL_RELOAD_SPEED  600.f   //3000.f
#define  ONESHOT_ANGLE  36860
#define  DIAL_WORK_TIME_MAX   300
//#define  FRIC_TEMPERATURE_MAX  

typedef enum{
	locked,
	unlock,

}Shoot_Safe_State_e;

typedef enum{
	CEASEFIRE,
	SIMGLE_SHOT,
	BURST,
	
}Shoot_Work_State_e;

typedef enum{
	LOAD_NO,
	LOAD_OK,

}Shoot_Load_State_e;

typedef enum{
//	DIAL_AWAKE,            //reset
	DIAL_SLEEP,
	DIAL_RELOAD,
	DIAL_RECOIL,

}Dial_Work_State_e;

typedef enum{
	DIAL_SPEED,
	DIAL_ANGLE,

}Dial_Mode_e;

typedef struct{
	Motor_RM_t* dial_config;
	Dial_Work_State_e dial_work_state;
	Dial_Mode_e dial_mode;
	float dial_speed_target;
	float dial_speed_fact;
	int32_t dial_angle_sum;
	int32_t dial_zero_angle_sum;
	uint8_t dial_work_time;
	uint16_t zero_start_reload_cnt; 
	uint16_t extra_angle;

}Dial_t;

typedef struct{
	Motor_RM_t* thr_fric[3];
	float fric_speed_target;
	float fric_speed_fact[3];
	float fric_speed_err[3];
	float fric_temperature_fact[3];
	
}Fric_t;

typedef struct{
	Motor_RM_t* pitch_motor;
	float pitch_angle_target;
	float p_k;
}Pitch_t;


typedef struct{
	Dial_t dial;
	Fric_t fric;
	Pitch_t pitch;
	Shoot_Safe_State_e shoot_safe_state;
	Shoot_Work_State_e shoot_work_state;
	Shoot_Load_State_e shoot_load_state;
	uint8_t awake_flag;
	uint8_t block_flag;
	uint8_t fric_ok_flag;
	uint8_t fire_flag;
	uint8_t firing_flag;
	uint16_t shoot_safe_cnt;
	uint8_t block_time;
	float k;
	
}Shoot_t;

extern Shoot_t shoot;

void Shoot_Init(Shoot_t *shoot);
void Shoot_Safe_State_Update(Shoot_t *shoot);
void Shoot_Work_State_Update(Shoot_t *shoot);
uint8_t Motor_Stuck_Check(Motor_RM_t* motor,uint16_t speed,uint16_t current,uint16_t stuck_time);
void Shoot_Reload(Shoot_t* shoot);
void Shoot_PID_Calculate(Shoot_t *shoot);
void Fric_State_Check(Shoot_t *shoot);
void Shoot_Sleep(Shoot_t *shoot);
void Shoot_Work(Shoot_t *shoot);


#endif
