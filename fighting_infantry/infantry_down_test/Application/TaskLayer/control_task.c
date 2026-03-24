
#include "control_task.h"
#include "Robot.h"

extern osSemaphoreId_t semTaskObserveToCtrl;
extern osSemaphoreId_t semTaskCtrlToObserve;

void StartCtrlTask(void const * argument)
{

	for(;;)
	{
		osSemaphoreAcquire(semTaskObserveToCtrl, osWaitForever);
		
		Command_Update();
		Chassis.status_react(&Chassis);
		
		Chassis.ctrl(&Chassis);
		
		Gimbal_Work(&gimbal);
    Launch_Work(&launch);
	
		Sd_Group.group_set_torque(&Sd_Group); 
//	  Chassis.Wheel->motor[R_WHEEL_M]->single_set_torque(Chassis.Wheel->motor[R_WHEEL_M]);
	  Chassis.Wheel->motor[L_WHEEL_M]->single_set_torque(Chassis.Wheel->motor[L_WHEEL_M]);
		CAN1_Set_Torque();
	
		osSemaphoreRelease(semTaskCtrlToObserve);
		osDelay(1);
	}
}


