/**
  ******************************************************************************
  * @file    Command_task.c
  * @brief   指令更新任务
  *          更新整车标志位和模式
  ******************************************************************************
  */
#include "Command_Task.h"

void StartCommandTask(void const * argument)
{
	for(;;)
	{
//		keyboard_update(rc_sensor.info);
		Balance.update(&Balance);
	
//    D_Board_Tx_Data(&D_Board_Tx_Pkt);		//板间通信
		D_Board_Tx1();
		D_Board_Tx2();
		D_Board_Tx3();
    D_Board_Tx4();
    D_Board_Tx5();
		
    osDelay(1);
	}
}  

