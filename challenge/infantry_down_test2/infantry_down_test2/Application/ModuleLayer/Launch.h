#ifndef  __LAUNCH_H
#define  __LAUNCH_H
#include "rm_motor.h"

typedef struct{
	float dial_angle_target;         
	float dial_speed_target;
	float dial_current_target; 
	uint8_t dial_work_state;
	uint8_t dial_mode;
	
}Launch_Rx_Info_t;

typedef struct{
	uint8_t Launch_state;
  uint8_t Launch_mode; 
  uint8_t is_fire;     
  uint8_t is_dial_online;
	
	float bullet_speed;
	float firing_freq; 
  float muzzle_temp; 
}Launch_Tx_Meg_t;


typedef struct{
	Motor_RM_t* dial;
	Launch_Rx_Info_t     info;
  Launch_Tx_Meg_t      meg;

}Launch_t;

extern Launch_t launch;



void Launch_Work(Launch_t* launch);



#endif
