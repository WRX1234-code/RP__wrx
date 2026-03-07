#include "Launch.h"
#include "shoot_Base.h"
#include "motor.h"
#include "Robot.h"
#include "Board_protocol.h"
#include "vision_protocol.h"

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
					.high_temp_speed_target = 9000,
					.normal_speed_target = 9050,
					.up_speed_target = 9050,
				  .speed_err_max = 150,
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
					.first_bullet_shield_time = 50,  
				  .high_adjust_speed = 0,
				  .high_adjust_value = 0.3,
				  .ideal_death_value = 0.01,
				  .ideal_speed_max = 24.8,
				  .ideal_speed_min = 24.6,
				  .less_cnt_max = 2,
				  .low_adjust_speed = 50,
				  .low_adjust_value = 0.3,
				  .more_cnt_max = 2,
				  .overspeed_adjust_speed = 80,
				  .speed_max = 25,
				  .speed_min = 24.4,
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

/*---------------------------------对外API定义-------------------------------------*/


void Launch_Board_Update(Launch_t* launch)
{
	//更新裁判系统数据用于弹速自适应和视觉需求
	launch->judge.now_speed = C_Board_Rx_Info.bullet_speed;   
	launch->judge.shoot_freq = C_Board_Rx_Info.firing_freq ;  
	launch->judge.muzzle_heat = C_Board_Rx_Info.muzzle_temp ; 
	launch->judge.muzzle_heat_max = C_Board_Rx_Info.muzzle_temp_max;
	launch->judge.bullet_allow = C_Board_Rx_Info.allow_bullet_cnt;
  
	//更新拨盘内容
	launch->base->info.rt_rx_info.dial_info.angle = C_Board_Rx_Info.dial_angle;
	launch->base->info.rt_rx_info.dial_info.speed = C_Board_Rx_Info.dial_speed;
	launch->base->info.rt_rx_info.dial_info.current = C_Board_Rx_Info.dial_current;
	
	C_Board_Tx_Pkt.dial_angle_target = launch->base->cmd.dial_tx_cmd.angle_sum_target;
	C_Board_Tx_Pkt.dial_speed_target = launch->base->cmd.dial_tx_cmd.speed_target;
	C_Board_Tx_Pkt.dial_current_target = launch->base->cmd.dial_tx_cmd.current_target;
	
	if(launch->base->cmd.dial_tx_cmd.mode == DIAL_ANGLE)
	{
		C_Board_Tx_Pkt.dial_mode = 0;
	}
	else if(launch->base->cmd.dial_tx_cmd.mode == DIAL_SPEED)
	{
		C_Board_Tx_Pkt.dial_mode = 1;
	}
	
	if(launch->base->work_state == LOCKED)
  {
		C_Board_Tx_Pkt.is_dial_need_sleep = 1;
	}
	else
	{
		C_Board_Tx_Pkt.is_dial_need_sleep = 0;
	}
	
	if(launch->base->work_state == UNLOCK && launch->base->cmd.dial_tx_cmd.work_state == SLEEP)
	{
		
	}
}


/**	
  * @brief    发射机构数据更新
  */	
void Launch_Data_Update(Launch_t* launch)
{
	Launch_Board_Update(launch);

	for(uint8_t i = 0; i < FRICTION_LIST;i++)
	{
		launch->info.fric_info.rt_rx_info[i].current = launch->assembly.group->motor[i]->rx_info->torque;
		launch->info.fric_info.rt_rx_info[i].speed = launch->assembly.group->motor[i]->rx_info->encoder_speed;
		launch->info.fric_info.rt_rx_info[i].temperature = launch->assembly.group->motor[i]->rx_info->temperature;
	}
	
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



void Vision_Tx_Update(Launch_t* launch)
{
	if(C_Board_Rx_Info.vision_mode != 0)
	{
		if(launch->base->cmd.vision_tx_cmd.is_ready_flag == 1)
	  {
		  vision_tx_frame.is_ready = 1;
	  
   	}
	  
	}
	
	vision_tx_frame.bullet_speed = C_Board_Rx_Info.bullet_speed;

}

void Self_Aim_Update(Launch_t* launch)
{
	if(C_Board_Rx_Info.vision_mode == 0)
	{
		return;
	}
	else{ 
		
		if(((vision_rx_frame.all_flags>>0) & 1) == 1)
		{
			
		}
	}
}

/**	
  * @brief    发射机构标志位更新
  * @note    更新的都是传入基础文件的标志位
  */	
/*需要修改*/    
void Launch_Flag_Update(Launch_t* launch)
{
	
	//更新is_sleep_flag
	if(robot.state == LOST)
	{
		launch->base->info.rt_rx_info.flag_Info.is_sleep_flag = 1;
	}
	else if((robot.state == RC_LIVE || robot.state == KEY_LIVE) && C_Board_Rx_Info.Launch_state == 0)
	{
		launch->base->info.rt_rx_info.flag_Info.is_sleep_flag = 1;
	}
	else if((robot.state == RC_LIVE || robot.state == KEY_LIVE) && C_Board_Rx_Info.Launch_state == 1)
	{
		launch->base->info.rt_rx_info.flag_Info.is_sleep_flag = 0;
	}
  
	
	//更新is_mtr_offline_flag
	if(launch->assembly.group->motor[FRICTION_UP]->state->status == DEV_OFFLINE 
		  || launch->assembly.group->motor[FRICTION_R]->state->status == DEV_OFFLINE
	    || launch->assembly.group->motor[FRICTION_L]->state->status == DEV_OFFLINE
	    || C_Board_Rx_Info.is_dial_online == 0)
	{
		launch->base->info.rt_rx_info.flag_Info.is_mtr_offline_flag = 1; 
	}
	else
	{
		launch->base->info.rt_rx_info.flag_Info.is_mtr_offline_flag = 0; 
	}

	//更新fire_mode_flag
	//非自瞄模式切换
	if(C_Board_Rx_Info.vision_mode == 0 || vision_cnt >= 70)
	{
		if(C_Board_Rx_Info.Launch_mode == 0)
	  {
		  launch->base->info.rt_rx_info.flag_Info.fire_mode_flag = 0;
	  }
		else if(C_Board_Rx_Info.Launch_mode == 1)
	  {
			launch->base->info.rt_rx_info.flag_Info.fire_mode_flag = 1;	
	  }
	}
	//自瞄相关
	else if(C_Board_Rx_Info.vision_mode != 0 && vision_cnt < 70)
	{
//		if(((vision_rx_frame.all_flags>>1)&1) == 1)
//	  {
//		  launch->base->info.rt_rx_info.flag_Info.fire_mode_flag = 1;
//		}
//		else if(((vision_rx_frame.all_flags>>1)&1) == 0)
//		{
//	    launch->base->info.rt_rx_info.flag_Info.fire_mode_flag = 0;
//	  }
		
		if(C_Board_Rx_Info.Launch_mode == 0)
	  {
		  launch->base->info.rt_rx_info.flag_Info.fire_mode_flag = 0;
	  }
		else if(C_Board_Rx_Info.Launch_mode == 1)
	  {
			launch->base->info.rt_rx_info.flag_Info.fire_mode_flag = 1;	
	  }
		
		
		
		
		
	}
  
	/*需要修改*/    
	//更新elec_level_flag，存储电平
	
	if(C_Board_Rx_Info.vision_mode == 0 || vision_cnt >= 70)
	{
		if(C_Board_Rx_Info.is_fire == 0)
	  {
	  	launch->base->info.rt_rx_info.flag_Info.elec_level_flag = 0;
	  }
	  else if(C_Board_Rx_Info.is_fire == 1)
	  {
		  launch->base->info.rt_rx_info.flag_Info.elec_level_flag = 1;
	  }
	}
	else if(C_Board_Rx_Info.vision_mode != 0 && vision_cnt < 70)
	{
//		if(((vision_rx_frame.all_flags>>2)&1) == 0)
//		{
//      launch->base->info.rt_rx_info.flag_Info.elec_level_flag = 0;
//    }			
//		else if(((vision_rx_frame.all_flags>>2)&1) == 1)
//		{
//			launch->base->info.rt_rx_info.flag_Info.elec_level_flag = 1;
//		}
		
		if(C_Board_Rx_Info.is_fire == 0)
	  {
	  	launch->base->info.rt_rx_info.flag_Info.elec_level_flag = 0;
	  }
	  else if(C_Board_Rx_Info.is_fire == 1)
	  {
		  launch->base->info.rt_rx_info.flag_Info.elec_level_flag = 1;
	  }
		
		
	}

	
	//外部更新run_limit_flag
	if(launch->flag.fric_block_flag == 1 || launch->flag.fric_high_temp_flag == 1 || launch->flag.fric_normal_speed_flag == 0)
	{
//		launch->base->info.rt_rx_info.flag_Info.run_limit_flag = 1;
	}
	else if(launch->base->info.rt_rx_info.flag_Info.is_sleep_flag == 1 || launch->base->info.rt_rx_info.flag_Info.is_mtr_offline_flag == 1)
	{
		launch->base->info.rt_rx_info.flag_Info.run_limit_flag = 1;
	}
	else if(C_Board_Rx_Info.muzzle_temp + 60 >= C_Board_Rx_Info.muzzle_temp_max)  //热量限制
	{
//		launch->base->info.rt_rx_info.flag_Info.run_limit_flag = 1;
	}
	else if(C_Board_Rx_Info.allow_bullet_cnt <= 0)   //允许发弹量
	{
//		launch->base->info.rt_rx_info.flag_Info.run_limit_flag = 1;
	}
	
	if(C_Board_Rx_Info.is_dial_self_reset == 1)
	{
		launch->base->info.rt_rx_info.flag_Info.init_flag = 0;
	}
	else if(C_Board_Rx_Info.is_dial_self_reset == 1)
	{
		launch->base->info.rt_rx_info.flag_Info.init_flag = 1;
	}
  
		
}


/**	
  * @brief    摩擦轮堵转处理
  */	
uint8_t Fric_Block_Check(Launch_t* launch)
{
	uint8_t flag = 0;
	for(uint8_t i = 0;i < FRICTION_LIST;i ++)
	{
		//阈值判断
		if(abs(launch->info.fric_info.rt_rx_info[i].speed) < launch->info.fric_info.cfg_rx_info.block_cfg[i].speed_max
		&& abs(launch->info.fric_info.rt_rx_info[i].current) > launch->info.fric_info.cfg_rx_info.block_cfg[i].current_min)
	  {
			if(launch->info.fric_info.cfg_rx_info.block_cfg[i].block_time>=launch->info.fric_info.cfg_rx_info.block_cfg[i].block_time_max)
			{
				flag = 1;
			  launch->flag.fric_block_flag= 1;     //堵转时更新堵转标志位
				launch->info.fric_info.cfg_rx_info.block_cfg[i].block_time = 0;				//这里清零可能有问题
			}
			else
			{
		  	launch->info.fric_info.cfg_rx_info.block_cfg[i].block_time ++;
			}
	  }
		else
		{
			launch->info.fric_info.cfg_rx_info.block_cfg[i].block_time = 0;         //不堵转时及时清零
		}
	}
	
	return flag;
}


/**
  * @brief   摩擦轮状态检查
  * @
  */


void Fric_State_Check(Launch_t* launch)
{
	
	int8_t k = 1;
	
	FRIC_SPEED_DATA_DIRECTION_MENAGE;      //用于处理k从而矫正摩擦轮转向
	
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
	
	#if IS_CHECK_DRIC_TEMP
	
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
	
	#endif
	
}

/**
  * @brief  弹速自适应
  * @note   可插入弹速，发弹延迟记录的函数
  */

void Launch_Speed_Self_Adapt(Launch_t* launch)
{
	static float speed;
	
	static uint16_t  more_cnt;                      
	static uint16_t  less_cnt;                      

	speed = launch->judge.now_speed;
	

	if(launch->judge.shoot_freq == 0)              //此时未初始化，不适应
	{
		more_cnt = 0;
		less_cnt = 0;
		return;
	}

	else if(launch->judge.shoot_freq > 0)          //检测到
	{
		static uint16_t tick;
    if (++tick < launch->info.fric_info.cfg_rx_info.adapt_cfg.first_bullet_shield_time)   //第一发弹由于加速，发弹延迟很大，需要屏蔽
		{
			more_cnt = 0;
			less_cnt = 0;
			return;
		}
    tick = 0;
	}
	
	//死区保证弹速不会在理想弹速区间临界处来回跳而导致反复适应，摩擦轮转速不稳
	if(speed >= (launch->info.fric_info.cfg_rx_info.adapt_cfg.ideal_speed_min - launch->info.fric_info.cfg_rx_info.adapt_cfg.ideal_death_value)   
	 && speed <= (launch->info.fric_info.cfg_rx_info.adapt_cfg.ideal_speed_max + launch->info.fric_info.cfg_rx_info.adapt_cfg.ideal_death_value))
	{
		more_cnt = 0;
		less_cnt = 0;
		return;
	}
	
  //超速
	if(speed >= launch->info.fric_info.cfg_rx_info.adapt_cfg.speed_max)
	{
		launch->info.fric_info.cfg_rx_info.base_cfg.normal_speed_target -= launch->info.fric_info.cfg_rx_info.adapt_cfg.overspeed_adjust_speed;
		more_cnt = 0;
		less_cnt = 0;
		
		return;
	}
  
	//低速
	if(speed < launch->info.fric_info.cfg_rx_info.adapt_cfg.ideal_speed_min)
	{
		less_cnt ++;               
		more_cnt = 0;
		
	  //低速需要次数超过阈值才能增速
		if(less_cnt >= launch->info.fric_info.cfg_rx_info.adapt_cfg.less_cnt_max)
		{
			less_cnt = 0;
		  
			//一阶线性增加弹速
			float low_add_speed = launch->info.fric_info.cfg_rx_info.adapt_cfg.low_adjust_value * launch->info.fric_info.cfg_rx_info.adapt_cfg.low_adjust_speed;
			
			if(low_add_speed <= 1)              //防止调整的速度太小
			{
				low_add_speed = 1;
			}
			launch->info.fric_info.cfg_rx_info.base_cfg.normal_speed_target += low_add_speed;
			return;
		}
	}
	
	//高速，不是超速，看情况减速
	if(speed > launch->info.fric_info.cfg_rx_info.adapt_cfg.ideal_speed_max && speed < launch->info.fric_info.cfg_rx_info.adapt_cfg.speed_max)
	{
		more_cnt ++;                      
		less_cnt = 0;
		
		if(more_cnt >= launch->info.fric_info.cfg_rx_info.adapt_cfg.more_cnt_max)
		{
			more_cnt = 0;
			
		  float divide = launch->info.fric_info.cfg_rx_info.adapt_cfg.speed_max - launch->info.fric_info.cfg_rx_info.adapt_cfg.ideal_speed_max;
		  
			if(divide == 0)   //被除数不为 0
		  {
		  	divide = 1;
	  	}
		
		  float high_minu_speed = launch->info.fric_info.cfg_rx_info.adapt_cfg.high_adjust_value * launch->info.fric_info.cfg_rx_info.adapt_cfg.high_adjust_speed;
		  if(high_minu_speed <= 1)
		  {
		  	high_minu_speed = 1;
		  }
			
			//按超出的比例来决定减掉的弹速
		  launch->info.fric_info.cfg_rx_info.base_cfg.normal_speed_target -= (speed - launch->info.fric_info.cfg_rx_info.adapt_cfg.ideal_speed_max)
		                                                      / divide *high_minu_speed;                   
	  }
		return;                             
	}
}

void Muzzle_Heat_Detect(Launch_t* launch)
{
	if(launch->judge.muzzle_heat_max - launch->judge.muzzle_heat < 30)
	{
		launch->base->info.rt_rx_info.flag_Info.run_limit_flag = 1;
	}
	else if(launch->judge.muzzle_heat_max - launch->judge.muzzle_heat >= 60 && launch->judge.muzzle_heat_max - launch->judge.muzzle_heat <= 30)
	{
		
	}

}

/**
  * @brief  摩擦轮PID计算
  * @note   拨盘在底盘，只给它发送目标值
  */
void Fric_Pid_Cal(Launch_t* launch)
{
	uint8_t i = 0;
	int8_t k = 1;
	if(launch->base->cmd.fric_tx_cmd.work_state == STOP)           //卸力时不算PID
	{
		for(i = 0;i< FRICTION_LIST;i++)
	  {
		  launch->assembly.tar.output = 0;
			
			launch->assembly.group->motor[i]->tx_info->torque = launch->assembly.tar.output;
	  }
	}

	else if(launch->base->cmd.fric_tx_cmd.work_state == RUN)	
	{
		for(i = 0;i< FRICTION_LIST;i++)
	  {
			if(i == 2)
			{
				k = -1;
			}
			else{
				k = 1;
			}
			
			if(i == 0)
			{
				launch->assembly.group->motor[i]->ctrl->speed_ctrl->target = launch->info.fric_info.cfg_rx_info.base_cfg.up_speed_target * k;
			}
			else{
			  launch->assembly.group->motor[i]->ctrl->speed_ctrl->target = launch->assembly.tar.speed_target * k;
			}
		  
		  launch->assembly.group->motor[i]->ctrl->speed_ctrl->measure = launch->assembly.group->motor[i]->rx_info->encoder_speed;
		  launch->assembly.group->motor[i]->ctrl->speed_ctrl->err = launch->assembly.group->motor[i]->ctrl->speed_ctrl->target 
		                                                         - launch->assembly.group->motor[i]->ctrl->speed_ctrl->measure;
		
		  single_pid_ctrl(launch->assembly.group->motor[i]->ctrl->speed_ctrl);
		  launch->assembly.tar.output = launch->assembly.group->motor[i]->ctrl->speed_ctrl->out;
			
			launch->assembly.group->motor[i]->tx_info->torque = launch->assembly.tar.output;
	  }
	}
}

/**
  * @brief  发射机构扭矩与信号发送
  * @note   拨盘在底盘，只给它发送目标值
  */
void Launch_Send(Launch_t* launch)
{
	launch->assembly.group->group_set_torque(launch->assembly.group);
	
}

/**
  * @brief   发射机构总控
  */

void Launch_Work(Launch_t* launch)
{
//	Fric_Block_Check(launch);           
	Fric_State_Check(launch);     
  Launch_Data_Update(launch);	
	Launch_Flag_Update(launch);
//	Launch_Speed_Self_Adapt(launch); 
	Shoot_Base_Work(launch->base);
	Fric_Pid_Cal(launch);
  Launch_Send(launch);
}
