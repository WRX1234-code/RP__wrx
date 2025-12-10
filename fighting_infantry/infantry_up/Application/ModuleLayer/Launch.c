#include "Launch.h"
#include "shoot_Base.h"
#include "motor.h"
#include "Robot.h"

Launch_t launch = {
	.base = &shoot,
	
	.assembly = {
		.group = &RM_Group,
		.tar = {
			.output = 0,
		  .speed_target = 0,
		},
	},
	
	.info = {
		.fric_info = {
		  .cfg_rx_info = {
				.base_cfg = {
					.high_temp_speed_target = 0,
					.normal_speed_target = 0,
				  .speed_err_max = 0,
				  .temp_err_max = 0,
				  .temp_max = 0,
				},
				.block_cfg = {
					[FRICTION_UP] = {
					  .block_time_max = 0,
				    .current_min = 0,
				    .menage_time_max = 0,
			      .speed_max = 0,
					},
					[FRICTION_R] = {
					  .block_time_max = 0,
				    .current_min = 0,
				    .menage_time_max = 0,
			      .speed_max = 0,
					},
					[FRICTION_L] = {
					  .block_time_max = 0,
				    .current_min = 0,
				    .menage_time_max = 0,
			      .speed_max = 0,
					},
					
				},
				.adapt_cfg = {
					.first_bullet_shield_time = 0,
				  .high_adjust_speed = 0,
				  .high_adjust_value = 0,
				  .ideal_death_value = 0,
				  .ideal_speed_max = 0,
				  .ideal_speed_min = 0,
				  .less_cnt_max = 0,
				  .low_adjust_speed = 0,
				  .low_adjust_value = 0,
				  .more_cnt_max = 0,
				  .overspeed_adjust_speed = 0,
				  .speed_max = 0,
				  .speed_min = 0,
				},
			}
		}	
	},
	
	.flag = {
		.fric_block_flag = 0,
		 .fric_high_temp_flag = 0,
	   .fric_normal_speed_flag = 1,
	}
};

void Launch_Data_Update(Launch_t* launch)
{
	for(uint8_t i = 0; i < FRICTION_LIST;i++)
	{
		launch->info.fric_info.rt_rx_info[i].current = launch->assembly.group->motor[i]->rx_info->torque;
		launch->info.fric_info.rt_rx_info[i].speed = launch->assembly.group->motor[i]->rx_info->encoder_speed;
		launch->info.fric_info.rt_rx_info[i].temperature = launch->assembly.group->motor[i]->rx_info->temperature;
	}

	launch->base->info.rt_rx_info.dial_info.angle = 0;
	launch->base->info.rt_rx_info.dial_info.current = 0;
	launch->base->info.rt_rx_info.dial_info.speed = 0;
	
	if(launch->base->cmd.fric_tx_cmd.work_state == STOP)
	{
		launch->assembly.tar.output = 0;
	}
	else if(launch->base->cmd.fric_tx_cmd.work_state == RUN)
	{
		if(launch->flag.fric_high_temp_flag == 1)
		{
			launch->assembly.tar.speed_target = launch->info.fric_info.cfg_rx_info.base_cfg.high_temp_speed_target;
		}
		else if(launch->flag.fric_block_flag == 1)
		{
			launch->assembly.tar.output = 0;
		}
		else
    {
			launch->assembly.tar.speed_target = launch->info.fric_info.cfg_rx_info.base_cfg.normal_speed_target;
		}
	}
}

void Launch_Flag_Update(Launch_t* launch)
{
	static float roller_step;
	static float last_roller_step;
	
	static float value;
	static float last_value;
	static float roller_value;
	
	static uint8_t key_cnt;
	
	roller_step=rc_sensor.info->thumbwheel.step[0];
	value=rc_sensor.info->s2.value;

	roller_value=rc_sensor.info->thumbwheel.value;
	roller_step=rc_sensor.info->thumbwheel.step[2];
	
	if((robot.CU==RC_CU&&roller_step!=last_roller_step)||(robot.CU==KEY_CU&&rc_sensor.info->V.status==release_to_press))
	{
		launch->misc.safe_cnt++;
	}
	
	if(robot.state == OFFLINE)
	{
		launch->base->info.rt_rx_info.flag_Info.is_sleep_flag = 1;
		
		launch->misc.safe_cnt=0;
	}
	else if(launch->misc.safe_cnt%2==0&&robot.state == ONLINE)
	{
		launch->base->info.rt_rx_info.flag_Info.is_sleep_flag = 1;
	}
	else if(launch->misc.safe_cnt%2==1&&robot.state == ONLINE)
	{
		launch->base->info.rt_rx_info.flag_Info.is_sleep_flag = 0;
	}

  if(robot.CU==RC_CU)
	{
		if(value==0x03&&(last_value==0x01||last_value==0x03))
	  {
			launch->base->info.rt_rx_info.flag_Info.fire_mode_flag = 0;
	  }
	  else if(value==0x01&&(last_value==0x03||last_value==0x01))
	  {
		  launch->base->info.rt_rx_info.flag_Info.fire_mode_flag = 1;
	  }
	  else
	  {
	  	launch->base->info.rt_rx_info.flag_Info.fire_mode_flag = 2;
	  }
		
		last_value=value;
	}
	else if(robot.CU == KEY_CU)
	{
		if(rc_sensor.info->B.status==release_to_press)
		{
			key_cnt++;
		}
		
		if(key_cnt%2==0)
		{
			launch->base->info.rt_rx_info.flag_Info.fire_mode_flag = 0;
		}
		else if(key_cnt%2==1)
		{
			launch->base->info.rt_rx_info.flag_Info.fire_mode_flag = 1;
		}
	
	}
	
	if(roller_value == 0 && roller_value == last_value)
	{
		launch->base->info.rt_rx_info.flag_Info.elec_level_flag = 0;
	}
	else if(roller_value > 0 && last_value > 0)
	{
		launch->base->info.rt_rx_info.flag_Info.elec_level_flag = 1;
	}
	
	last_roller_step=roller_step;
	last_value = roller_value;
}



uint8_t Fric_Block_Check(Launch_t* launch)
{
	uint8_t flag = 0;
	for(uint8_t i = 0;i < FRICTION_LIST;i ++)
	{
		if(abs(launch->info.fric_info.rt_rx_info[i].speed) < launch->info.fric_info.cfg_rx_info.block_cfg[i].speed_max
		&& abs(launch->info.fric_info.rt_rx_info[i].current) > launch->info.fric_info.cfg_rx_info.block_cfg[i].current_min)
	  {
			if(launch->info.fric_info.cfg_rx_info.block_cfg[i].block_time>=launch->info.fric_info.cfg_rx_info.block_cfg[i].block_time_max)
			{
				flag = 1;
			  launch->flag.fric_block_flag= 1;
			}
			else
			{
		  	launch->info.fric_info.cfg_rx_info.block_cfg[i].block_time ++;
			}
	  }
		else
		{
			launch->info.fric_info.cfg_rx_info.block_cfg[i].block_time = 0;
		}
	}
	
	return flag;
}



void Fric_State_Check(Launch_t* launch)
{
	
	int8_t k = 1;
	
	FRIC_SPEED_DATA_DIRECTION_MENAGE
	
	uint8_t i;
	for(i = 0;i < FRICTION_LIST;i ++)                
	{
		launch->assembly.check.speed_err[i] = abs(launch->info.fric_info.rt_rx_info[i].speed - k*launch->info.fric_info.cfg_rx_info.base_cfg.normal_speed_target);
		launch->assembly.check.temp_err[i] = launch->info.fric_info.rt_rx_info[i].temperature -launch->info.fric_info.cfg_rx_info.base_cfg.temp_max;
	}
	
	#if FRIC_NUM == 6             
	
	if(launch->assembly.check.speed_err[FRIC_B_UP] < launch->info.fric_info.cfg_rx_info.base_cfg.speed_err_max
		&& launch->assembly.check.speed_err[FRIC_B_R] < launch->info.fric_info.cfg_rx_info.base_cfg.speed_err_max
    && launch->assembly.check.speed_err[FRIC_B_L] < launch->info.fric_info.cfg_rx_info.base_cfg.speed_err_max
    && launch->assembly.check.speed_err[FRIC_F_UP] < launch->info.fric_info.cfg_rx_info.base_cfg.speed_err_max
    && launch->assembly.check.speed_err[FRIC_F_R] < launch->info.fric_info.cfg_rx_info.base_cfg.speed_err_max
    && launch->assembly.check.speed_err[FRIC_F_L] < launch->info.fric_info.cfg_rx_info.base_cfg.speed_err_max	)

  #elif FRIC_NUM == 3	           
	if(launch->assembly.check.speed_err[FRIC_UP] < launch->info.fric_info.cfg_rx_info.base_cfg.speed_err_max
		&& launch->assembly.check.speed_err[FRIC_R] < launch->info.fric_info.cfg_rx_info.base_cfg.speed_err_max
	  && launch->assembly.check.speed_err[FRIC_L] < launch->info.fric_info.cfg_rx_info.base_cfg.speed_err_max)

  #elif FRIC_NUM == 2	           
	if(launch->assembly.check.speed_err[FRIC_R] < launch->info.fric_info.cfg_rx_info.base_cfg.speed_err_max
		&& launch->assembly.check.speed_err[FRIC_L] < launch->info.fric_info.cfg_rx_info.base_cfg.speed_err_max)
	#endif
	{
    launch->flag.fric_normal_speed_flag = 1;
	}
	else 
	{
		launch->flag.fric_normal_speed_flag = 0;
	}	
	
	
	#if FRIC_NUM == 6                
	
	if(launch->assembly.check.temp_err[FRIC_B_UP] < launch->info.fric_info.cfg_rx_info.base_cfg.temp_err_max
		&& launch->assembly.check.temp_err[FRIC_B_R] < launch->info.fric_info.cfg_rx_info.base_cfg.temp_err_max
    && launch->assembly.check.temp_err[FRIC_B_L] < launch->info.fric_info.cfg_rx_info.base_cfg.temp_err_max
    && launch->assembly.check.temp_err[FRIC_F_UP] < launch->info.fric_info.cfg_rx_info.base_cfg.temp_err_max
    && launch->assembly.check.temp_err[FRIC_F_R] < launch->info.fric_info.cfg_rx_info.base_cfg.temp_err_max
    && launch->assembly.check.temp_err[FRIC_F_L] < launch->info.fric_info.cfg_rx_info.base_cfg.temp_err_max	)

  #elif FRIC_NUM == 3	             
	if(launch->assembly.check.temp_err[FRIC_UP] < launch->info.fric_info.cfg_rx_info.base_cfg.temp_err_max
		&& launch->assembly.check.temp_err[FRIC_R] < launch->info.fric_info.cfg_rx_info.base_cfg.temp_err_max
	  && launch->assembly.check.temp_err[FRIC_L] < launch->info.fric_info.cfg_rx_info.base_cfg.temp_err_max)

  #elif FRIC_NUM == 2	             
	if(launch->assembly.check.temp_err[FRIC_R] < launch->info.fric_info.cfg_rx_info.base_cfg.temp_err_max
		&& launch->assembly.check.temp_err[FRIC_L] < launch->info.fric_info.cfg_rx_info.base_cfg.temp_err_max)
	#endif
	{
		launch->flag.fric_high_temp_flag = 0;
	}
	else 
	{
		launch->flag.fric_high_temp_flag = 1;
	}	
	
}


 
void launch_Speed_Self_Adapt(Launch_t* launch)
{
	static float speed;
	
	static uint16_t  more_cnt;                      
	static uint16_t  less_cnt;                      

	speed = launch->judge.now_speed;
	

	if(launch->judge.shoot_freq == 0)              
	{
		more_cnt = 0;
		less_cnt = 0;
		return;
	}

	else if(launch->judge.shoot_freq > 0)
	{
		static uint16_t tick;
    if (++tick < launch->info.fric_info.cfg_rx_info.adapt_cfg.first_bullet_shield_time) 
		{
			more_cnt = 0;
			less_cnt = 0;
			return;
		}
    tick = 0;
	}
	
	
	if(speed >= (launch->info.fric_info.cfg_rx_info.adapt_cfg.ideal_speed_min - launch->info.fric_info.cfg_rx_info.adapt_cfg.ideal_death_value)   //0.05f??????????????
	 && speed <= (launch->info.fric_info.cfg_rx_info.adapt_cfg.ideal_speed_max + launch->info.fric_info.cfg_rx_info.adapt_cfg.ideal_death_value))
	{
		more_cnt = 0;
		less_cnt = 0;
		return;
	}
	

	if(speed >= launch->info.fric_info.cfg_rx_info.adapt_cfg.speed_max)
	{
		launch->info.fric_info.cfg_rx_info.base_cfg.normal_speed_target -= launch->info.fric_info.cfg_rx_info.adapt_cfg.overspeed_adjust_speed;
		more_cnt = 0;
		less_cnt = 0;
		
		return;
	}

	if(speed < launch->info.fric_info.cfg_rx_info.adapt_cfg.ideal_speed_min)
	{
		less_cnt ++;               
		more_cnt = 0;
		
	
		if(less_cnt >= launch->info.fric_info.cfg_rx_info.adapt_cfg.less_cnt_max)
		{
			less_cnt = 0;
		
			float low_add_speed = launch->info.fric_info.cfg_rx_info.adapt_cfg.low_adjust_value * launch->info.fric_info.cfg_rx_info.adapt_cfg.low_adjust_speed;
			if(low_add_speed <= 1)
			{
				low_add_speed = 1;
			}
			launch->info.fric_info.cfg_rx_info.base_cfg.normal_speed_target += low_add_speed;
			return;
		}
	}
	
	
	if(speed > launch->info.fric_info.cfg_rx_info.adapt_cfg.ideal_speed_max && speed < launch->info.fric_info.cfg_rx_info.adapt_cfg.speed_max)
	{
		more_cnt ++;                      
		less_cnt = 0;
		
		if(more_cnt >= launch->info.fric_info.cfg_rx_info.adapt_cfg.more_cnt_max)
		{
			more_cnt = 0;
			
		  float divide = launch->info.fric_info.cfg_rx_info.adapt_cfg.speed_max - launch->info.fric_info.cfg_rx_info.adapt_cfg.ideal_speed_max;
		  if(divide == 0)
		  {
		  	divide = 1;
	  	}
		
		  float high_minu_speed = launch->info.fric_info.cfg_rx_info.adapt_cfg.high_adjust_value * launch->info.fric_info.cfg_rx_info.adapt_cfg.high_adjust_speed;
		  if(high_minu_speed <= 1)
		  {
		  	high_minu_speed = 1;
		  }
		    launch->info.fric_info.cfg_rx_info.base_cfg.normal_speed_target -= (speed - launch->info.fric_info.cfg_rx_info.adapt_cfg.ideal_speed_max)
		                                                      / divide *high_minu_speed;                   
	  }
		return;                             
	}
}

void Fric_Pid_Cal(Launch_t* launch)
{
	if(launch->assembly.tar.output != 0)
	{
		for(uint8_t i = 0;i< FRICTION_LIST;i++)
	  {
		  launch->assembly.group->motor[i]->ctrl->speed_ctrl->target = launch->assembly.tar.speed_target;
		  launch->assembly.group->motor[i]->ctrl->speed_ctrl->measure = launch->assembly.group->motor[i]->rx_info->encoder_speed;
		  launch->assembly.group->motor[i]->ctrl->speed_ctrl->err = launch->assembly.group->motor[i]->ctrl->speed_ctrl->target 
		                                                         - launch->assembly.group->motor[i]->ctrl->speed_ctrl->measure;
		
		  single_pid_ctrl(launch->assembly.group->motor[i]->ctrl->speed_ctrl);
		  launch->assembly.tar.output = launch->assembly.group->motor[i]->ctrl->speed_ctrl->out;
	  }
	}
}



//void Shoot_Sleep(Shoot_t* shoot)
//{

//	shoot->fric.cmd.work_state =STOP;
//	shoot->fric.cmd.output = 0;
//	shoot->dial.cmd.current = 0;
//	shoot->dial.cmd.work_state = SLEEP;

//}


void Launch_Work(Launch_t* launch)
{
	Fric_Block_Check(launch);           
	Fric_State_Check(launch);           
	Shoot_Speed_Self_Adapt(launch); 
	Shoot_Base_Work(launch->base);

}
