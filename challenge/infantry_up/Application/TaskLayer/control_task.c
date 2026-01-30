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
		Gimbal_Work(&gimbal);
		Launch_Work(&launch);
		
		
		osDelay(1);
	}
}



