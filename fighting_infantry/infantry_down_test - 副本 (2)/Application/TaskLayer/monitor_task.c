/**
  ******************************************************************************
  * @file    monitor_task.c
  * @brief   监控任务
  *          1. 各模块心跳失联检测
  *          2. 监控遥控器状态，软件复位
  ******************************************************************************
  */
#include "monitor_task.h"
#include "device.h"
#include "priority_ui.h"
#include "ui.h"
#include "ui_protocol.h"

uint16_t open_ui = 0;
void StartMonitorTask(void const * argument)
{
	
	for(;;)
	{
		rc_sensor.heart_beat(&rc_sensor);
		imu_sensor.heart_beat(&imu_sensor.work_state);
	//	Sd_Group.group_heartbeat(&Sd_Group);	  //在Chassis.heartbeat里
	//	Wheel_Group.group_heartbeat(&Wheel_Group);//在Chassis.heartbeat里
		
		Chassis.heartbeat(&Chassis);
		
		Yaw_Motor.single_heart_beat(&Yaw_Motor);
		Dial_Motor.single_heart_beat(&Dial_Motor);
		
		Judge_Heart_Beat(&judge);
		cap.heartbeat(&cap);
		D_Board_Heart_Beat();
		
		Cmd_Heartbeat();
		
		if(open_ui == 0)
	  {
			My_Ui_Init();
			open_ui = 1;
			
	  }
	  else
	  {
		  Ui_Info_Update();
		
		  Ui_Send();
		
	  }
		
		HAL_IWDG_Refresh(&hiwdg1);
		
		
		osDelay(1);
	}
}

