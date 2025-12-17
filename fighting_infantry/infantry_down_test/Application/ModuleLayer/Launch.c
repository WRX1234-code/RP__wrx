#include "Launch.h"
#include "Board_protocol.h"
#include "Launch_Motor.h"
#include "judge.h"
#include "rp_math.h"

Launch_t launch = {
	.dial = &Dial_Motor,

};


void Launch_Rx_Meg_Update(Launch_t* launch)
{
	launch->info.dial_angle_target = D_Board_Rx_Info.dial_angle_target;
	launch->info.dial_speed_target = D_Board_Rx_Info.dial_speed_target;
	launch->info.dial_current_target = D_Board_Rx_Info.dial_current_target;
	
}

void Launch_Tx_Meg_Update(Launch_t* launch)
{
	if(launch->dial->state->status == DEV_ONLINE)
	{
		D_Board_Tx_Pkt.is_dial_online = 1;
	}
	else
	{
		D_Board_Tx_Pkt.is_dial_online = 0;
	}
	
	D_Board_Tx_Pkt.dial_angle = launch->dial->rx_info->encoder_sum;
	D_Board_Tx_Pkt.dial_speed = launch->dial->rx_info->encoder_speed;
	D_Board_Tx_Pkt.dial_current = launch->dial->rx_info->torque;
	
	D_Board_Tx_Pkt.bullet_speed = judge.info->shoot_data.initial_speed;
	D_Board_Tx_Pkt.firing_freq = judge.info->shoot_data.launching_frequency;
	D_Board_Tx_Pkt.muzzle_temp = judge.info->power_heat_data.shooter_17mm_1_barrel_heat;
	
}

void Launch_Pid_Cal(Launch_t* launch)
{
	if(D_Board_Rx_Info.is_dial_need_sleep == 1)
	{
		launch->dial->tx_info->torque = launch->info.dial_current_target;
	}
	
	else if(D_Board_Rx_Info.dial_mode == 0 && D_Board_Rx_Info.is_dial_need_sleep == 0)
	{
		launch->dial->ctrl->angle_ctrl_outer->target = launch->info.dial_angle_target;
		launch->dial->ctrl->angle_ctrl_outer->measure = launch->dial->rx_info->encoder_sum;
		launch->dial->ctrl->angle_ctrl_outer->err = launch->dial->ctrl->angle_ctrl_outer->target - launch->dial->ctrl->angle_ctrl_outer->measure;
	  launch->dial->ctrl->angle_ctrl_outer->err = half_cycle((float)launch->dial->ctrl->angle_ctrl_outer->err,8192.f);
	  
		single_pid_ctrl(launch->dial->ctrl->angle_ctrl_outer);
		
		launch->dial->ctrl->angle_ctrl_inner->target = launch->dial->ctrl->angle_ctrl_outer->out;
		launch->dial->ctrl->angle_ctrl_inner->measure = launch->dial->rx_info->encoder_speed;
		launch->dial->ctrl->angle_ctrl_inner->err = launch->dial->ctrl->angle_ctrl_inner->target - launch->dial->ctrl->angle_ctrl_inner->measure;
	  
		single_pid_ctrl(launch->dial->ctrl->angle_ctrl_inner);
		
		launch->dial->tx_info->torque = launch->dial->ctrl->angle_ctrl_inner->out;
	}           
  else if(D_Board_Rx_Info.dial_mode == 1 && D_Board_Rx_Info.is_dial_need_sleep == 0)	
	{
		launch->dial->ctrl->speed_ctrl->target = launch->info.dial_speed_target;
		launch->dial->ctrl->speed_ctrl->measure = launch->dial->rx_info->encoder_speed;
		launch->dial->ctrl->speed_ctrl->err = launch->dial->ctrl->speed_ctrl->target - launch->dial->ctrl->speed_ctrl->measure;
		
		single_pid_ctrl(launch->dial->ctrl->speed_ctrl);
		
		launch->dial->tx_info->torque = launch->dial->ctrl->speed_ctrl->out;
	}                                       
}

void Launch_Send(Launch_t* launch)
{
	if(D_Board_Rx_Info.is_dial_need_sleep == 1)
	{
		launch->dial->single_sleep(launch->dial);
	}
	else if(D_Board_Rx_Info.is_dial_need_sleep == 0)
	{
		launch->dial->single_set_torque(launch->dial);
	}
}
	

void Launch_Work(Launch_t* launch)
{
	Launch_Rx_Meg_Update(launch);
	Launch_Pid_Cal(launch);
	Launch_Send(launch);
	Launch_Tx_Meg_Update(launch);
	
}






