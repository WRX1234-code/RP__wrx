/**
 ******************************************************************************
 * @file    monitor_task.c
 * @brief   监控任务
 *          1. 各模块心跳失联检测
 *          2. 监控遥控器状态，软件复位
 ******************************************************************************
 */
#include "monitor_task.h"

int16_t a;
void StartMonitorTask(void const *argument)
{


	for (;;)
	{
		rm_motor_list_heart_beat();
		Yaw_Motor.single_heart_beat(&Yaw_Motor);
		L_Wheel.single_heart_beat(&L_Wheel);
		
		osDelay(1);
	}
}

