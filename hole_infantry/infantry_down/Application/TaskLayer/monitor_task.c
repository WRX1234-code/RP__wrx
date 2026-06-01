/**
 ******************************************************************************
 * @file    monitor_task.c
 * @brief   监控任务
 *          1. 各模块心跳失联检测
 *          2. 监控遥控器状态，软件复位
 ******************************************************************************
 */
#include "monitor_task.h"
#include "board_protocol.h"
#include "imu_sensor.h"
#include "cap.h"
#include "judge.h"
void StartMonitorTask(void const *argument)
{

	for (;;)
	{
		rm_motor_list_heart_beat();
		rc_sensor.heart_beat(&rc_sensor);
		imu_sensor.heart_beat(&imu_sensor.work_state);
		cap.heartbeat(&cap);
		board.heartbeat(&board);
		judge.heartbeat(&judge);
	
		osDelay(1);
	}
}

