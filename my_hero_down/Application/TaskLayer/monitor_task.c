/**
 ******************************************************************************
 * @file    monitor_task.c
 * @brief   监控任务
 *          1. 各模块心跳失联检测
 *          2. 监控遥控器状态，软件复位
 ******************************************************************************
 */
#include "monitor_task.h"

#include "Chassis.h"

int16_t a;
void StartMonitorTask(void const *argument)
{


	for (;;)
	{
//		rm_motor_list_heart_beat();
//		Yaw_Motor.single_heart_beat(&Yaw_Motor);
//		L_Wheel.single_heart_beat(&L_Wheel);
		
//		Chassis_Heart_Beat(&chassis_wheel);
		
//		rm_motor_heart_beat(&chassis_wheel.wheel_lf);
//		
//		void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
//		
		osDelay(1);
	}
}

