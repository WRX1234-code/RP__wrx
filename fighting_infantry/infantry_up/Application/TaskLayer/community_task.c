#include "community_task.h"
#include "vision_protocol.h"
#include "Vision.h"
#include "Board_protocol.h"
#include "imu_sensor.h"
#include "bmi.h"
#include "Robot.h"

void StartCommunityTask(void const *argument)
{
	

	for (;;)
	{
		
		if(imu_sensor.work_state.err_code == IMU_NONE_ERR ||
			imu_sensor.work_state.err_code == IMU_DATA_CALI)
		{
			imu_sensor.update(&imu_sensor);
		}
		Robot_State_Update(&robot);
		
		Vision_Data_Update();
		Vision_Tx_data(&vision_tx_frame);
		
    osDelay(1);
	}
}

