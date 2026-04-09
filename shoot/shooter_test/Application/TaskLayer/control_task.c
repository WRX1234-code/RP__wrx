/**
  ******************************************************************************
  * @file    control_task.c
  * @brief   
  ******************************************************************************
  */
#include "control_task.h"
#include "Shooter.h"

float t;
void StartControlTask(void const * argument)
{
	Shoot_Init(&shoot);

	for(;;) 
	{
		Shoot_Work(&shoot);
		osDelay(1);
	}
}



