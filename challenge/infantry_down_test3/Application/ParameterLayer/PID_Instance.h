#ifndef __PID_Instance_H
#define __PID_Instance_H

#include "PID.h"
#include "car_info.h"


typedef struct Link_Pid_struct_t
{
	pid_ctrl_t* roll_cal[Leg_Num];
	
	pid_ctrl_t* sync_cal[Leg_Num];//Ë«ÍÈÐ­µ÷
	
	pid_ctrl_t* length_cal[Leg_Num];
	pid_ctrl_t* length_speed_cal[Leg_Num];
	
	pid_ctrl_t* yaw_cal[Leg_Num];
	pid_ctrl_t* yaw_speed_cal[Leg_Num];
	pid_ctrl_t* vir_phi0_cal[Leg_Num];
	pid_ctrl_t* vir_phi0_speed_cal[Leg_Num];
	pid_ctrl_t* vir_phi0d1_cal[Leg_Num];
}Chassis_Pid_t;


extern Chassis_Pid_t chassis_PID;

#endif


