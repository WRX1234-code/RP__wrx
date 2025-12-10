#ifndef __MOTOR_H
#define __MOTOR_H

#include "rp_config.h"
#include "can_protocol.h"
#include "rm_motor.h"
#include "KT_motor.h"
#include "HT_motor.h"
#include "DM_motor.h"
#include "motor_def.h"
#include "drv_can.h"
#include "pid.h"

/*电机ID宏定义------------------------------------------------*/
#define ID_GIMB_P 		0x206 //0x1FF  23


extern Motor_RM_t Fric_Up_Motor;
extern Motor_RM_t	Fric_R_Motor;
extern Motor_RM_t	Fric_L_Motor;

extern  Motor_DM_t Pitch_Motor;

extern  Motor_RM_Group_t RM_Group;

typedef enum {
	
	FRIC_UP,	//		CAN1	 0x201
  FRIC_R,	//		CAN1	 0x202
  FRIC_L,	//  	CAN1	 0x203
	//GIMB_P, 	//		CAN1	 0x206
	
	//IMAGE,        //	CAN1	 0x206
	//TELESCOPE,    //	CAN1	 0x205
	
	FRIC_MOTOR_LIST,
} dev_fric_motor_list_e;			 
									  

/* Exported functions --------------------------------------------------------*/
void rm_motor_list_init(void);
void rm_motor_list_heart_beat(void);
void kt_motor_list_init(void);
void ht_motor_list_init(void);
void dm_motor_list_init(void);
uint8_t rm_motor_list_workstate(void);

#endif

