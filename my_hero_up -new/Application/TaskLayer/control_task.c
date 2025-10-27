/**
  ******************************************************************************
  * @file    control_task.c
  * @brief   
  ******************************************************************************
  */
#include "control_task.h"
#include "Gimbal.h"
#include "Shooter.h"

float t;
void StartControlTask(void const * argument)
{
	
	

	for(;;) 
	{
//		Yaw_Motor.tx_info->torque = t;
//		Yaw_Motor.single_set_torque(&Yaw_Motor);
		imu_sensor.update(&imu_sensor);
		Gimbal_Drive(&gimbal_motor);
		Shoot_Work(&shoot);
		
		osDelay(1);
	}
}



