#include "community_task.h"
#include "communicate.h"
#include "Gimbal.h"
void StartCommunityTask(void const *argument)
{
	
	
	for (;;)
	{
		
		Communicate_Chassis_Message(&communicate_chassis_target);
		Gimbal_motor_Send();

		osDelay(1);
	}
}
