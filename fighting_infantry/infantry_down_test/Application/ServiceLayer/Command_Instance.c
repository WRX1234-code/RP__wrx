
#include "Command_Instance.h"

command_t command[COMMAND_LIST] = 
{
  [JUMP] = {
    .cmd_type = RISE_TRIGER_C,
    .run_time_max = OUT_TIME_OFF,  
	.init = Cmd_Class_Init,
	},
  [KNEE_STRIKE] = {
    .cmd_type = RISE_TRIGER_C,
    .run_time_max = OUT_TIME_OFF,  
	.init = Cmd_Class_Init,
	},

};

/**
 * @brief 命令初始化，调用一次
 */
void Cmd_Init(void)
{
	for(uint8_t i = 0; i < COMMAND_LIST; i++)
	{
		command[i].init(&command[i]);
	}
}
/**
 * @brief 命令心跳 循环调用
 */
void Cmd_Heartbeat(void)
{
	for(uint8_t i = 0; i < COMMAND_LIST; i++)
	{
		command[i].heartbeat(&command[i]);
	}
}
/**
 * @brief 命令更新 循环调用
 */
void Command_Update(void)
{
	static uint32_t RC_ONLINE_TICK;
	rc_sensor_info_t*  rc_info=Balance.rc->sensor->info;
	
	if(Balance.rc->sensor->work_state==DEV_ONLINE)
	{
		RC_ONLINE_TICK++;
	}
	else
	{
		RC_ONLINE_TICK=0;
	}
	static uint8_t last_rc_info_s1;
	
	if(RC_ONLINE_TICK>=200)//屏蔽开控命令
	{
		/*命令更新填这里*/
		command[JUMP].update(&command[JUMP],last_rc_info_s1==RC_SW_MID&&
										rc_info->s1==RC_SW_UP);
		
		command[KNEE_STRIKE].update(&command[KNEE_STRIKE],last_rc_info_s1==RC_SW_MID&&
										rc_info->s1==RC_SW_DOWN);
		
		
	}
	
	
	last_rc_info_s1=rc_info->s1;
}



