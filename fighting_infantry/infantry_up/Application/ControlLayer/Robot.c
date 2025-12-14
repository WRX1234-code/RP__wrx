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

void Robot_STATE_Update(Robot_t* robot)
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

void Robot_Mode_Update(Robot_t* robot)
{
	if(C_Board_Rx_Info.car_base_mode == 0)
	{
		robot->mode.base_mode = GYRO;
	}
	else if(C_Board_Rx_Info.car_base_mode == 1)
	{
		robot->mode.base_mode = S_GYRO;
	}
	else if(C_Board_Rx_Info.car_base_mode == 2)
	{
		robot->mode.base_mode = MEC;
	}
	
	if(C_Board_Rx_Info.car_adv_mode == 0)
	{
		robot->mode.adv_mode = H_S_S_GYRO;
	}
	else if(C_Board_Rx_Info.car_adv_mode == 1 || C_Board_Rx_Info.vision_mode == 1)
	{
		robot->mode.adv_mode = SELF_AIM;
	}
	else if(C_Board_Rx_Info.car_adv_mode == 2)
	{
		robot->mode.adv_mode = MELEE;
	}
	else if(C_Board_Rx_Info.car_adv_mode == 3)
	{
		robot->mode.adv_mode = SUSPEND;
	}
	else if(C_Board_Rx_Info.car_adv_mode == 4)
	{
		robot->mode.adv_mode = DAFU;
	}
};

//void Robot_Mode_Switch(Robot_t* robot)
//{
//	switch (robot->mode.base_mode)
//	{
//		case MEC:
//			robot->mode.mode_switch = SUSPEND;
//		  break;
//		
//		case GYRO:
//			robot->mode.mode_switch = ()
//		
//	}
//}	

void Robot_Cmd_Update(Robot_t* robot)
{
	if(C_Board_Rx_Info.car_cmd == 0)
	{
		robot->cmd = NO_CMD;
	}
	if(C_Board_Rx_Info.car_cmd == 1)
	{
		robot->cmd = U_TURN;
	}
	else if(C_Board_Rx_Info.car_cmd == 2)
	{
		robot->cmd = TURN_L_45;
	}
	else if(C_Board_Rx_Info.car_cmd == 3)
	{
		robot->cmd = TURN_R_45;
	}
	else if(C_Board_Rx_Info.car_cmd == 4)
	{
		robot->cmd =JUMP;
	}
	else if(C_Board_Rx_Info.car_cmd == 5)
	{
		robot->cmd = KNEE_UP;
	}
	else if(C_Board_Rx_Info.car_cmd == 6)
	{
		robot->cmd = FLY;
	}
	else if(C_Board_Rx_Info.car_cmd == 7)
	{
		robot->cmd = REVERSE_FLY;
	}
	
}

//void Robot_Elec_Level_Update(){
//	static float roller_step;
//	static float last_roller_step;
//	
//	static float last_value;
//	static float roller_value;
//	
//	static uint8_t key_cnt;
//	
//	roller_step=rc_sensor.info->thumbwheel.step[2];
//	roller_value=rc_sensor.info->thumbwheel.value;
//	
//	if((robot.CU == RC_CU && (roller_step != last_roller_step && roller_value > 0))
//		||(robot.CU == KEY_CU && rc_sensor.info->mouse_btn_l.status == release_to_press))
//	{
//		robot.elec_level = RISING_EDGE;
//	}
//		
//	else if((robot.CU == RC_CU &&(roller_step == last_roller_step && roller_value>0))
//		     ||(robot.CU == KEY_CU &&(rc_sensor.info->mouse_btn_l.status == release_to_press
//		     || rc_sensor.info->mouse_btn_l.status == short_press || rc_sensor.info->mouse_btn_l.status == long_press)))
//	{
//		robot.elec_level = HIGH_LEVEL;		
//	}
//	else if((robot.CU == RC_CU && (roller_value < last_value && roller_value > 0 && last_value < 660))
//		     ||(robot.CU == KEY_CU && rc_sensor.info->mouse_btn_l.status == press_to_release))
//	{
//		robot.elec_level = FALLING_EDGE;
//	}
//	else if((robot.CU == RC_CU && roller_value == 0 && last_value == 0) || (robot.CU == KEY_CU && rc_sensor.info->mouse_btn_l.status == release))
//	{
//		robot.elec_level = LOW_LEVEL;
//	}
//		 
//	last_roller_step=roller_step;
//	last_value = roller_value;
//};


void Robot_Cmd_Excute(Robot_t* robot)
{
	switch (robot->cmd)
	{
		case NO_CMD:
			break;
		
		case U_TURN:
			C_Board_Rx_Info.yaw_imu_tar += 180.f;
		  break;
		default:
			break;
			
	}
}

