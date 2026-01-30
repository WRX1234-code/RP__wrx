#include "connect_task.h"
#include "board_protocol.h"

void StartConnectTask(void const * argument)
{

	for(;;) 
	{
	  C_Board_Tx1();
		C_Board_Tx2();
    C_Board_Tx3();
		
		
		osDelay(2);
	}
}