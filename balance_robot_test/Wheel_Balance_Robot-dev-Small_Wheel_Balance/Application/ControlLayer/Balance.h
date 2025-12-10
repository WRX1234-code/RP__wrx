#ifndef __BALANCE_H
#define __BALANCE_H

#include "rc_sensor.h"
#include "chassis.h"
#include "Command_Instance.h"
#define BALANCE_INIT_CNT_MAX 800

typedef enum
{
	RC_CTRL = 0,
	KEY_CTRL,
}Balance_Ctrl_e;
typedef enum
{
	Balance_reset_NO,
	Balance_reset_OK,
	
}Balance_reset_state_e;

typedef struct Balance_reset_state_struct_t
{
	Balance_reset_state_e reset_state;
	uint16_t reset_cnt;
}Balance_reset_state_t;

typedef enum
{
	Init_Mode = 0,
	Sleep_Mode,
	Imu_Mode,
	Mec_Mode,
	LEG_TEST_Mode,
}Balance_Mode_e;

typedef struct Balance_Flag_struct_t
{
	bool Clear_Flag;
	bool Chassis_Online_Flag;
	bool Chassis_Sleep_Flag;
	bool Knee_Strike_Flag;
	bool Rescue_Flag;
	bool Rescue_Trigger;
	bool Unable_Rescue_Flag;//无法自救
	bool Leg_length_ctrl_Flag;
	bool Jumping_Flag;//跳跃过程中，用于给chassis状态信号量
	uint8_t Rescue_step;
	
}Balance_Flag_t;

typedef struct Balance_Remote_Ctrl_struct_t
{
	rc_sensor_t* sensor;
	uint8_t* last_thumbwheel_step;
}Balance_Remote_Ctrl;

typedef struct Balance_struct_t
{
	//Balance_Ctrl_e ctrl;
	
	Balance_Mode_e mode;
	
	Balance_Mode_e last_mode;
	
	Balance_Remote_Ctrl* rc;
	
	Balance_reset_state_t reset_struct;
	
	Balance_Flag_t* Flag;
	
	command_t* command;
	
	void(*init)(struct Balance_struct_t* balance);
	
	void(*update)(struct Balance_struct_t* balance);
}Balance_t;

extern Balance_t Balance;







#endif
