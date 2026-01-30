#include "Robot.h"
#include "rc_sensor.h"
#include "board_protocol.h"
#include "vision_protocol.h"
Robot_t robot = {
	.mode = {
		.mode_switch = 0,
		.self_aim_flag = 0,
	},
};

void Robot_State_Update(Robot_t* robot)
{
	if(C_Board_Rx_Info.car_state == 0)
	{
		robot->state = LOST;
	}
  else if(C_Board_Rx_Info.car_state == 1)
	{
		robot->state = RC_LIVE;
	}
	 else if(C_Board_Rx_Info.car_state == 2)
	{
		robot->state = KEY_LIVE;
	}
};


