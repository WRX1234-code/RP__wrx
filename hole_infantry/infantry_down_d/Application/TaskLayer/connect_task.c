#include "connnect_task.h"
#include "board_protocol.h"

void StartConnectTask(void const * argument)
{

	for(;;)
	{
		board.tx_01(&board);
		board.tx_02(&board);
		board.tx_03(&board);
		board.tx_04(&board);
		
		osDelay(1);
	}
}