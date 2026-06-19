#include "community_task.h"
#include "rc_protocol.h"
#include "board_protocol.h"
#include "bmi.h"

void StartCommunityTask(void const *argument)
{

	for (;;)
	{
		BMI_Change_Kp(1000,0.125);//陀螺仪Kp初始化很大，后面很小
		
		rc_interrupt_update(&rc_sensor);
    keyboard_update(rc_sensor.info); // 键鼠状态检测

		board.tx_01(&board);
		board.tx_02(&board);
		board.tx_03(&board);
		board.tx_04(&board);
		
		
	
		osDelay(1);
	}
}
