/**
  ******************************************************************************
  * @file    control_task.c
  * @brief   
  ******************************************************************************
  */
#include "control_task.h"

float t;
void StartControlTask(void const * argument)
{

	for(;;) 
	{
		Yaw_Motor.tx_info->torque = t;
		Yaw_Motor.single_set_torque(&Yaw_Motor);
		osDelay(1);
	}
}



