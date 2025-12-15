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
	Turn_Mode,
	LEG_TEST_Mode,
	//VISION_TEST_mode,
	Key_Mode,
}Balance_Mode_e;

typedef struct Balance_Flag_struct_t
{
	bool Clear_Flag;
	bool Chassis_Online_Flag;
	bool Chassis_Sleep_Flag;
	
	bool Rescue_Flag;
	bool Rescue_Trigger;
	bool Unable_Rescue_Flag;//无法自救
	uint8_t Rescue_step;
	
	bool Leg_length_ctrl_Flag;
	
	bool Turn_Flag;
	bool S_Turn_Flag;
	
	bool Launch_Open_Flag;
	bool Self_Aim_Flag;//自瞄
	bool Vision_Test_Flag;//视觉调试，腿卸力
	bool Key_Flag;
	
	bool Jumping_Flag;//跳跃过程中，用于给chassis状态信号量
	bool Knee_Strike_Flag;
	bool Fly_Flag;
	bool Reserve_Fly_Flag;
	
	bool U_Turn_Flag;
	bool R_Turn_Flag;
	bool L_Turn_Flag;

	bool Lob_Flag;
	uint16_t Auto_step;   //内含打小符，打大符，打前哨
	uint16_t Fly_step;
	
	bool Heat_Limit_Flag;
	
}Balance_Flag_t;

typedef struct Balance_Remote_Ctrl_struct_t
{
	rc_sensor_t* sensor;
	uint8_t* last_thumbwheel_step;
	uint8_t last_s2;
}Balance_Remote_Ctrl;

typedef struct Balance_struct_t
{
	Balance_Ctrl_e ctrl;
	
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
