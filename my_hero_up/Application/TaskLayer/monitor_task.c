/**
 ******************************************************************************
 * @file    monitor_task.c
 * @brief   监控任务
 *          1. 各模块心跳失联检测
 *          2. 监控遥控器状态，软件复位
 ******************************************************************************
 */
#include "monitor_task.h"
#include "rc_sensor.h"
#include "communicate.h"
#include "Gimbal.h"
#include "Shooter.h"

int16_t a;
void StartMonitorTask(void const *argument)
{


	for (;;)
	{
//		rm_motor_list_heart_beat();
//		Yaw_Motor.single_heart_beat(&Yaw_Motor);
//		L_Wheel.single_heart_beat(&L_Wheel);
		
		
//		if(heart_cnt>70)
//		{
//			heart_cnt=70;
//			communicate_chassis_target.heart_state=1;
////			gimbal_heart_state=1;
//		}
//		else
//		{
//			communicate_chassis_target.heart_state=0;
//		} 
		
//		if(rc_sensor.info->s2.value==0x02)
//		{
//			
//		}
		
		
		if(rc_sensor_info.s1.value==0x03)
		{
			gimbal_motor.gimbal_mode=1;
		}
		else if(rc_sensor_info.s1.value==0x02)
		{
			gimbal_motor.gimbal_mode=2;
		}
		else if(rc_sensor_info.s1.value==0x01)
		{
			gimbal_motor.gimbal_mode=3;
		}
		
		
		Gimbal_Heart_Beat(&gimbal_motor);
		
		osDelay(1);
	}
}

