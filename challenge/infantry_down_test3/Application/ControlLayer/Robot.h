#ifndef  __ROBOT_H
#define  __ROBOT_H

#include "stdint.h"

/*需要修改*/  
/*需要升级*/  
/*需要放下板*/
/*需要删除*/    
/*需要保存*/    
/*需要思考*/   

//typedef enum{
//	GYRO =0,
//	S_GYRO,
//	MEC,
//	BASE_CNT,
//}Robot_Base_Mode_e;

/*需要升级*/
//typedef enum{
//	H_S_S_GYRO   = 1u << 0,
//  SELF_AIM     = 1u << 1,
//	MELEE        = 1u << 2,
//	SUSPEND      = 1u << 3,
//	DAFU         = 1u << 4,
//	
//}Robot_Adv_Mode_e;     

//typedef enum{
//	NO_ADV_MODE = 0, 
//	H_S_S_GYRO, 
//  SELF_AIM,     
//	MELEE,     
//	SUSPEND,     
//	DAFU,      
//	ADV_MODE_CNT,
//}Robot_Adv_Mode_e;

///*需要升级*/
//typedef struct{
//	Robot_Base_Mode_e     base_mode;
//  Robot_Adv_Mode_e      adv_mode;
//	uint8_t               self_aim_flag;
//	
//	uint32_t              mode_switch;
//	
//	
//}Robot_Mode_e;

///*需要放下板*/
//typedef enum{
//	NO_CMD,
//	U_TURN,
//	TURN_L_45,
//	TURN_R_45,
//	JUMP,
//	KNEE_UP,
//  FLY,
//  REVERSE_FLY,
//	ARREST_HERO,
//	ARREST_EBGINE,
//	CMD_CNT,
//	
//}Robot_Cmd_t;

//typedef enum{
//	LOST,
//	RC_LIVE,
//	KEY_LIVE,
//	
//	STATE_CNT,
//}Robot_State_e;

///*需要思考*/
//typedef enum{
//	RISING_EDGE = 0,
//	FALLING_EDGE,
//	HIGH_LEVEL,
//	LOW_LEVEL,

//}Robot_Elec_Level_e;


//typedef struct{
//	Robot_Mode_e          mode;
//	Robot_Cmd_t           cmd;
//	Robot_State_e         state;
//  Robot_Elec_Level_e    elec_level;
//	
//}Robot_t;

//extern Robot_t robot;

//void Robot_STATE_Update(Robot_t* robot);
//void Robot_Mode_Update(Robot_t* robot);
//void Robot_Cmd_Update(Robot_t* robot);
//void Robot_Cmd_Excute(Robot_t* robot);

#endif
