/**
  ******************************************************************************
  * @file    control_task.c
  * @brief   
  ******************************************************************************
  */
#include "control_task.h"
#include "Chassis.h"
#include "cap.h"
float t;
void StartControlTask(void const * argument)
{
	
  Chassis_Wheel_Init(&chassis_wheel);
	
	for(;;) 
	{
//		Yaw_Motor.tx_info->torque = t;
//		Yaw_Motor.single_set_torque(&Yaw_Motor);
		
		Chassis_Remote_Receive(&chassis_wheel);
		Chassis_Speed_Calculate(&chassis_wheel);
		Chassis_PID_Speed_Calculate(&chassis_wheel);
		New_Chassis_Power_Limit(chassis_wheel);

		Chassis_Send(&chassis_wheel);
		cap.setdata(&cap,60,80);
		
		CAP_txMessage();
		osDelay(1);
	}
}



