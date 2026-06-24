/**
  ******************************************************************************
  * @file    control_task.c
  * @brief   
  ******************************************************************************
  */
#include "control_task.h"
#include "cap.h"
#include "ui.h"
#include "priority_ui.h"
#include "infantry.h"

uint8_t open_ui = 0;
void StartCtrlTask(void const * argument)
{

	for(;;) 
	{
		infantry.work(&infantry);
		cap.tx();
		
		if(open_ui == 0)
	  {
			open_ui = 1;
			
	  }
	  else
	  {
		  Ui_Info_Update();
		
		  Ui_Send();
		
	  }
		osDelay(1);
	}
}



