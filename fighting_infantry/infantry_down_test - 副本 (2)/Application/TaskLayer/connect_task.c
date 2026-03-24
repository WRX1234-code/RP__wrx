#include "connnect_task.h"
#include "board_protocol.h"

void StartConnectTask(void const * argument)
{

	for(;;)
	{
		D_Board_Tx1();
		D_Board_Tx2();
		D_Board_Tx3();
    D_Board_Tx4();
    D_Board_Tx5();
		osDelay(2);
	}
}