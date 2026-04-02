/**
 ******************************************************************************
 * @file    monitor_task.c
 * @brief   监控任务
 *          1. 各模块心跳失联检测
 *          2. 监控遥控器状态，软件复位
 ******************************************************************************
 */
#include "monitor_task.h"
#include "rc_protocol.h"
#include "vision_protocol.h"
#include "Board_protocol.h"
#include "bmi.h"

float init_kp = 1000.f;
float now_kp = 0.125f;
void StartMonitorTask(void const *argument)
{


	for (;;)
	{
		BMI_Change_Kp(init_kp,now_kp);
		
		rm_motor_list_heart_beat();
		Pitch_Motor.single_heart_beat(&Pitch_Motor);
		
//		rc_sensor.heart_beat(&rc_sensor);
//		rc_sensor.check(&rc_sensor);		//拨轮拨杆跳变判断、数据异常检查
//		rc_interrupt_update(&rc_sensor);   //鼠标值均值滤波
//		keyboard_update(rc_sensor.info); // 键鼠状态检测
		
		imu_sensor.heart_beat(&imu_sensor.work_state);
	  Board_Heart_Beat();
		Vision_heart_beat();
		
		osDelay(1);
	}
}

