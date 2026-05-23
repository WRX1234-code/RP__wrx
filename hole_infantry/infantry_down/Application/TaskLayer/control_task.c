/**
  ******************************************************************************
  * @file    control_task.c
  * @brief   
  ******************************************************************************
  */
#include "control_task.h"
#include "cap.h"

float t;
void StartControlTask(void const * argument)
{

	for(;;) 
	{
		cap.tx();
		osDelay(1);
	}
}



