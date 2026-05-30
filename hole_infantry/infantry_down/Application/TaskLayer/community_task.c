#include "community_task.h"
#include "rc_protocol.h"
#include "board_protocol.h"
#include "bmi.h"

void StartCommunityTask(void const *argument)
{

	for (;;)
	{
		BMI_Change_Kp(1000,0.125);
		
		rc_interrupt_update(&rc_sensor);
    keyboard_update(rc_sensor.info); // ¼üÊó×´Ì¬¼ì²â

		board.tx_01(&board);
		board.tx_02(&board);
		board.tx_03(&board);
		board.tx_04(&board);
		
		
	
		osDelay(1);
	}
}
