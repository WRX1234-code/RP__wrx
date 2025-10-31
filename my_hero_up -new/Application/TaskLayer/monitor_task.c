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
		rc_sensor.check(&rc_sensor);		//拨轮拨杆跳变判断、数据异常检查
		rc_interrupt_update(&rc_sensor);   //鼠标值均值滤波
		keyboard_update(rc_sensor.info); // 键鼠状态检测
		
		if(rc_sensor.info->s2.value==0x02)
		{
			communicate_control_mode=KEY_MODE;
		}
		else if(rc_sensor.info->s2.value==0x01||rc_sensor.info->s2.value==0x03)
		{
			communicate_control_mode=RC_MODE;
		}
		
		gimbal_motor.last_gimbal_mode=gimbal_motor.gimbal_mode;
		
		if((communicate_control_mode==RC_MODE&&rc_sensor_info.s1.value==0x03)||(communicate_control_mode==KEY_MODE&&rc_sensor.info->Z.status==release_to_press))
		{
			gimbal_motor.gimbal_mode=1;
		}
		else if((communicate_control_mode==RC_MODE&&rc_sensor_info.s1.value==0x02)||(communicate_control_mode==KEY_MODE&&rc_sensor.info->X.status==release_to_press))
		{
			gimbal_motor.gimbal_mode=2;
		}
		else if((communicate_control_mode==RC_MODE&&rc_sensor_info.s1.value==0x01)||(communicate_control_mode==KEY_MODE&&rc_sensor.info->C.status==release_to_press))
		{
			gimbal_motor.gimbal_mode=3;
		}
		
		
		Gimbal_Heart_Beat(&gimbal_motor);
		
		osDelay(1);
	}
}

