/**
  ******************************************************************************
  * @file    Command_task.c
  * @brief   指令更新任务
  *          更新整车标志位和模式
  ******************************************************************************
  */
#include "Command_Task.h"

void StartCommandTask(void const * argument)
{
	for(;;)
	{
//		keyboard_update(rc_sensor.info);
		Balance.update(&Balance);
		
		osDelay(1);
	}
}
