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
#include "bmi.h"

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
		while(HAL_GetTick() <= 200)
		{}
		bmi.Kp = 0.125;

		
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
		
		
		gimbal_motor.last_heart_state=gimbal_motor.gimbal_heart_state;
		
		Gimbal_Heart_Beat(&gimbal_motor);
		
		
		
		if(gimbal_motor.gimbal_heart_state==0&&gimbal_motor.last_heart_state==1)
		{
			gimbal_motor.restore_gimbal_mode=gimbal_motor.gimbal_mode;
			gimbal_motor.gimbal_mode=1;
			gimbal_motor.init_zero_flag=1;
			osDelay(1000);
		}
		else if(gimbal_motor.gimbal_heart_state==0&&gimbal_motor.init_zero_flag==1&&gimbal_motor.gimbal_mode==1&&gimbal_motor.gimbal_y_motor.y_motor->KT_motor_info.rx_info.speed==0&&abs(gimbal_motor.gimbal_y_motor.y_motor->KT_motor_info.rx_info.current)<=10)
		{
//			Gyro_zero_bias(&gimbal_motor);
			gimbal_motor.gimbal_mode=gimbal_motor.restore_gimbal_mode;
			gimbal_motor.init_zero_flag=0;
		}
		else if(gimbal_motor.init_zero_flag==0)
		{
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
		
		}
		
		
		
		
		
		
		osDelay(1);
	}
}

