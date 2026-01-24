#ifndef __CHASSIS_MOTOR_H
#define __CHASSIS_MOTOR_H

#include "DM_Motor.h"
#include "RM_Motor.h"
#include "drv_can.h"
#include "kalman_filter.h"



#define ID_R_F_Sd_M    0x11
#define ID_R_B_Sd_M    0x12
#define ID_L_F_Sd_M    0x13
#define ID_L_B_Sd_M    0x14

typedef enum
{

	R_F_Sd_M,
	R_B_Sd_M,
	L_F_Sd_M,
	L_B_Sd_M,
	Sd_Num,
	
}Motor_Sd_e;

typedef enum
{
	R_WHEEL_M,
	L_WHEEL_M,
	Wheel_Num,
}Motor_Wheel_e;
extern Motor_DM_Group_t Sd_Group;
extern Motor_RM_Group_t Wheel_Group;
extern Motor_RM_t Test_Motor;

#endif
