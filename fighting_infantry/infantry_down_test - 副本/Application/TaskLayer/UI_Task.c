/**
  ******************************************************************************
  * @file    UI_task.c
  * @brief   UI更新任务
  ******************************************************************************
  */
#include "Ui_Task.h"

float sin_value;
uint32_t t;

__attribute__((section (".AXI_SRAM"))) uint8_t  t_buf[8]={0x12,0x12,0x12,0x12,0x12,0x12,0x12,0x12};


void StartUITask(void const * argument)
{
		for(;;)
	{
		
	
		
		osDelay(1);
	}
}
 

