#include "Launch.h"
#include "shoot_Base.h"
#include "motor.h"
#include "Robot.h"
#include "Board_protocol.h"
#include "vision_protocol.h"
#include "RM_Motor.h"
#include "drv_can.h"
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
					.high_temp_speed_target = 6200,
					.normal_speed_target = 6200,
					.up_speed_target = 6200,
				  .speed_err_max = 200,
				  .temp_err_max = 0,
				  .temp_max = 0,
				},
				.block_cfg = {
					[FRICTION_R] = {
					  .block_time_max = 50,
				    .current_min = 6000,
				    .menage_time_max = 0,
			      .speed_max = 100,
					},
					[FRICTION_L] = {
					  .block_time_max = 50,
				    .current_min = 6000,
				    .menage_time_max = 0,
			      .speed_max = 100,
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
	
	if(launch->base->cmd.dial_tx_cmd.work_state == RELOAD)
	{
		C_Board_Tx_Pkt.is_hit_now = 1;
	}
	else{
	  C_Board_Tx_Pkt.is_hit_now = 0;
	}
}


/**	
  * @brief    发射机构数据更新
  */	
void Launch_Data_Update(Launch_t* launch)
{
	Launch_Board_Update(launch);

//	if(launch->base->cmd.dial_tx_cmd.mode == DIAL_ANGLE)
//	{
//		 launch->info.fric_info.cfg_rx_info.base_cfg.normal_speed_target = 6200;
//	}
//	else if(launch->base->cmd.dial_tx_cmd.mode == DIAL_SPEED)
//	{
//		launch->info.fric_info.cfg_rx_info.base_cfg.normal_speed_target = 6350;
//	}
	
	for(uint8_t i = 0; i < FRICTION_LIST;i++)
	{
		launch->info.fric_info.rt_rx_info[i].current = launch->assembly.group->motor[i]->rx_info->torque_current_raw;
		launch->info.fric_info.rt_rx_info[i].speed = launch->assembly.group->motor[i]->rx_info->encoder_speed;
		launch->info.fric_info.rt_rx_info[i].temperature = launch->assembly.group->motor[i]->rx_info->temperature;
	}
	
	if(launch->base->cmd.fric_tx_cmd.work_state == STOP)
	{
		launch->assembly.tar.output = 0;
		launch->assembly.tar.speed_target = 0;
	}
	else if(launch->base->cmd.fric_tx_cmd.work_state == RUN)
	{
		if(launch->flag.fric_high_temp_flag == 1)
		{
			launch->assembly.tar.speed_target = launch->info.fric_info.cfg_rx_info.base_cfg.high_temp_speed_target;
		}
		else if(launch->flag.fric_block_flag == 1)
		{
//			launch->assembly.tar.output = 0;
		}
		else
    {
			launch->assembly.tar.speed_target = launch->info.fric_info.cfg_rx_info.base_cfg.normal_speed_target;
		}
	}
}



void Vision_Tx_Update(Launch_t* launch)
{
	
	if(launch->base->cmd.vision_tx_cmd.is_ready_flag == 1)
  {
		vision_tx_frame.is_ready = 1;
	  
  }
  else if(launch->base->cmd.vision_tx_cmd.is_ready_flag == 0)
  {
	  vision_tx_frame.is_ready = 0;
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
	static uint8_t last_dial_reset = 0;
	
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
	if(launch->assembly.group->motor[FRICTION_R]->state->status == DEV_OFFLINE
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
	if(C_Board_Rx_Info.vision_mode == 0 || C_Board_Tx_Pkt.vision_state == 0)
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
	else if(C_Board_Rx_Info.vision_mode != 0 && C_Board_Tx_Pkt.vision_state == 1)
	{
		if(robot.state == RC_LIVE)
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
		else if(robot.state == KEY_LIVE)
		{
			if(((vision_rx_frame.all_flags>>1)&1) == 1)
	    {
		    launch->base->info.rt_rx_info.flag_Info.fire_mode_flag = 1;
		  }
		  else if(((vision_rx_frame.all_flags>>1)&1) == 0)
		  {
	      launch->base->info.rt_rx_info.flag_Info.fire_mode_flag = 0;
	    }
		}
		
//		if(C_Board_Rx_Info.Launch_mode == 0)
//	  {
//		  launch->base->info.rt_rx_info.flag_Info.fire_mode_flag = 0;
//	  }
//		else if(C_Board_Rx_Info.Launch_mode == 1)
//	  {
//			launch->base->info.rt_rx_info.flag_Info.fire_mode_flag = 1;	
//	  }
		
	}
  
	/*需要修改*/    
	//更新elec_level_flag，存储电平
	
	if(C_Board_Rx_Info.vision_mode == 0 || C_Board_Tx_Pkt.vision_state == 0)
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
	else if(C_Board_Rx_Info.vision_mode != 0 && C_Board_Tx_Pkt.vision_state == 1)
	{
		if(robot.state == RC_LIVE)
		
		{
//			if(C_Board_Rx_Info.is_fire == 0)
//	    {
//	  	  launch->base->info.rt_rx_info.flag_Info.elec_level_flag = 0;
//	    }
//	    else if(C_Board_Rx_Info.is_fire == 1)
//	    {
//		    launch->base->info.rt_rx_info.flag_Info.elec_level_flag = 1;
//	    }
			
			if(C_Board_Rx_Info.vision_mode != 2 && C_Board_Rx_Info.vision_mode != 3)
		  {
			  if(C_Board_Rx_Info.is_fire == 1 && (((vision_rx_frame.all_flags>>2)&1) == 1))
		    {
			    launch->base->info.rt_rx_info.flag_Info.elec_level_flag = 1;
		    }
		    else{
		      launch->base->info.rt_rx_info.flag_Info.elec_level_flag = 0;
		    }
		  }
		  else{
			  if(((vision_rx_frame.all_flags>>2)&1) == 0)
		    {
          launch->base->info.rt_rx_info.flag_Info.elec_level_flag = 0;
        }			
		    else if(((vision_rx_frame.all_flags>>2)&1) == 1)
		    {
			    launch->base->info.rt_rx_info.flag_Info.elec_level_flag = 1;
		    }
		  }

		}
		else if(robot.state == KEY_LIVE)
		{
			if(C_Board_Rx_Info.vision_mode != 2 && C_Board_Rx_Info.vision_mode != 3)
		  {
			  if(C_Board_Rx_Info.is_fire == 1 && (((vision_rx_frame.all_flags>>2)&1) == 1))
		    {
			    launch->base->info.rt_rx_info.flag_Info.elec_level_flag = 1;
		    }
		    else{
		      launch->base->info.rt_rx_info.flag_Info.elec_level_flag = 0;
		    }
		  }
		  else{
			  if(((vision_rx_frame.all_flags>>2)&1) == 0)
		    {
          launch->base->info.rt_rx_info.flag_Info.elec_level_flag = 0;
        }			
		    else if(((vision_rx_frame.all_flags>>2)&1) == 1)
		    {
			    launch->base->info.rt_rx_info.flag_Info.elec_level_flag = 1;
		    }
		  }
		}
		
	}

	
	//外部更新run_limit_flag
//	if(launch->flag.fric_block_flag == 1 || launch->flag.fric_high_temp_flag == 1 || launch->flag.fric_normal_speed_flag == 0)
//	{
//		launch->base->info.rt_rx_info.flag_Info.run_limit_flag = 1;
//	}
	if(launch->base->info.rt_rx_info.flag_Info.is_sleep_flag == 1 || launch->base->info.rt_rx_info.flag_Info.is_mtr_offline_flag == 1)
	{
		launch->base->info.rt_rx_info.flag_Info.run_limit_flag = 1;
	}
	else{
	  launch->base->info.rt_rx_info.flag_Info.run_limit_flag = 0;
	}
//	if(C_Board_Rx_Info.muzzle_temp + 60 >= C_Board_Rx_Info.muzzle_temp_max)  //热量限制
//	{
//		launch->base->info.rt_rx_info.flag_Info.run_limit_flag = 1;
//		
//	}
	Muzzle_Heat_Detect(launch);
	if(C_Board_Rx_Info.allow_bullet_cnt <= 5)   //允许发弹量
	{
//		launch->base->info.rt_rx_info.flag_Info.run_limit_flag = 1;
	}
	
	if(C_Board_Rx_Info.is_dial_self_reset != last_dial_reset && launch->base->info.rt_rx_info.flag_Info.init_flag == 1)
	{
		launch->base->info.rt_rx_info.flag_Info.init_flag = 0;
	}
	
	last_dial_reset = C_Board_Rx_Info.is_dial_self_reset;
		
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
//				launch->info.fric_info.cfg_rx_info.block_cfg[i].block_time = 0;				//这里清零可能有问题
				launch->info.fric_info.cfg_rx_info.block_cfg[i].block_time=launch->info.fric_info.cfg_rx_info.block_cfg[i].block_time_max;
				
				return flag;
			}
			else
			{
				flag = 0;
				launch->flag.fric_block_flag= 0; 
		  	launch->info.fric_info.cfg_rx_info.block_cfg[i].block_time ++;
			}
	  }
		else
		{
			flag = 0;
			launch->flag.fric_block_flag= 0; 
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
	
	uint8_t i;
	for(i = 0;i < FRICTION_LIST;i ++)                
	{
//		FRIC_SPEED_CORRECT(k);      //用于处理k从而矫正摩擦轮转向
		
		if(i == 0)
		{
			k = -1;
		}
		else{
		  k = 1;
		}
		
		launch->assembly.check.speed_err[i] = abs(launch->info.fric_info.rt_rx_info[i].speed - k*launch->info.fric_info.cfg_rx_info.base_cfg.normal_speed_target);
		launch->assembly.check.temp_err[i] = launch->info.fric_info.rt_rx_info[i].temperature -launch->info.fric_info.cfg_rx_info.base_cfg.temp_max;
	}
	
	#if FRIC_NUMBER == 6             
	
	if(launch->assembly.check.speed_err[FRICTION_B_UP] < launch->info.fric_info.cfg_rx_info.base_cfg.speed_err_max
		&& launch->assembly.check.speed_err[FRICTION_B_R] < launch->info.fric_info.cfg_rx_info.base_cfg.speed_err_max
    && launch->assembly.check.speed_err[FRICTION_B_L] < launch->info.fric_info.cfg_rx_info.base_cfg.speed_err_max
    && launch->assembly.check.speed_err[FRICTION_F_UP] < launch->info.fric_info.cfg_rx_info.base_cfg.speed_err_max
    && launch->assembly.check.speed_err[FRICTION_F_R] < launch->info.fric_info.cfg_rx_info.base_cfg.speed_err_max
    && launch->assembly.check.speed_err[FRICTION_F_L] < launch->info.fric_info.cfg_rx_info.base_cfg.speed_err_max	)

  #elif FRIC_NUMBER == 3	           
	if(launch->assembly.check.speed_err[FRICTION_UP] < launch->info.fric_info.cfg_rx_info.base_cfg.speed_err_max
		&& launch->assembly.check.speed_err[FRICTION_R] < launch->info.fric_info.cfg_rx_info.base_cfg.speed_err_max
	  && launch->assembly.check.speed_err[FRICTION_L] < launch->info.fric_info.cfg_rx_info.base_cfg.speed_err_max)

  #elif FRIC_NUMBER == 2	           
	if(launch->assembly.check.speed_err[FRICTION_R] < launch->info.fric_info.cfg_rx_info.base_cfg.speed_err_max
		&& launch->assembly.check.speed_err[FRICTION_L] < launch->info.fric_info.cfg_rx_info.base_cfg.speed_err_max)
	#endif
	{
    launch->flag.fric_normal_speed_flag = 1;
	}
	else 
	{
		launch->flag.fric_normal_speed_flag = 0;
	}	
	
	#if IS_CHECK_DRIC_TEMP
	
	  #if FRIC_NUMBER == 6                
	
	    if(launch->assembly.check.temp_err[FRIC_B_UP] < launch->info.fric_info.cfg_rx_info.base_cfg.temp_err_max
		    && launch->assembly.check.temp_err[FRIC_B_R] < launch->info.fric_info.cfg_rx_info.base_cfg.temp_err_max
        && launch->assembly.check.temp_err[FRIC_B_L] < launch->info.fric_info.cfg_rx_info.base_cfg.temp_err_max
        && launch->assembly.check.temp_err[FRIC_F_UP] < launch->info.fric_info.cfg_rx_info.base_cfg.temp_err_max
        && launch->assembly.check.temp_err[FRIC_F_R] < launch->info.fric_info.cfg_rx_info.base_cfg.temp_err_max
        && launch->assembly.check.temp_err[FRIC_F_L] < launch->info.fric_info.cfg_rx_info.base_cfg.temp_err_max	)

    #elif FRIC_NUMBER == 3	             
	    if(launch->assembly.check.temp_err[FRIC_UP] < launch->info.fric_info.cfg_rx_info.base_cfg.temp_err_max
	  	  && launch->assembly.check.temp_err[FRIC_R] < launch->info.fric_info.cfg_rx_info.base_cfg.temp_err_max
	      && launch->assembly.check.temp_err[FRIC_L] < launch->info.fric_info.cfg_rx_info.base_cfg.temp_err_max)

    #elif FRIC_NUMBER == 2	             
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

///*弹速自适应*/
//int8_t adapt_cnt = 0, low_cnt = 0;

//float last_measure_speed = 0.f;
//float limit_Shoot_Speed = 25.f;

//int8_t long_minus_step = -13;
//int8_t short_minus_step = -4;
//int8_t short_plus_step = 4;
//int8_t long_plus_step = 10;

//int8_t Adapt_k = 4;
//static void  My_Fric_Speed_Adapt(Launch_t* launch)//目标平均速度24.6f （24.7.24.5）
//{
//	static int8_t Adapt_Speed;
//	float launch_speed_now ;//= My_Judge.org_info->shoot_data.bullet_speed;
//	
//	launch_speed_now = launch->judge.now_speed;
//	if(board_cnt < 70)
//	{
//		
//	if((last_measure_speed != launch->judge.now_speed) && (launch->judge.now_speed > 0))
//	{
//		if(launch_speed_now >= (limit_Shoot_Speed - 0.1f))//24.9以上
//		{
//			low_cnt = 0;
//			adapt_cnt = 0;
//			Adapt_Speed = long_minus_step;
//		}
//		else if(launch_speed_now > (limit_Shoot_Speed - 0.3f))//24.7以上
//		{
//			low_cnt = 0;
//			adapt_cnt++;
//			if(adapt_cnt >= 2)
//			{
//				Adapt_Speed = short_minus_step;
//				adapt_cnt = 0;
//			}
//		}
//		else if(launch_speed_now < (limit_Shoot_Speed - 0.5f))//24.5以下
//		{
//			adapt_cnt--;
//			low_cnt = 0;
//			if(adapt_cnt<=-2)
//			{
//				Adapt_Speed = short_plus_step;
//				adapt_cnt = 0;
//			}
//		}
//		else if(launch_speed_now <= (limit_Shoot_Speed - 0.8f))//24.2以下
//		{
//			low_cnt ++;
//			if(low_cnt >= 3)
//			{
//				Adapt_Speed = long_plus_step;
//			}
//			
//		}
//		else
//		{
//			adapt_cnt = 0;
//			low_cnt = 0;
//			Adapt_Speed = 0;
//		}
//		
//		launch->info.fric_info.cfg_rx_info.base_cfg.normal_speed_target += Adapt_Speed*Adapt_k;
//		
//	}
//	last_measure_speed = launch->judge.now_speed;
// }
//	else
//	{
//		last_measure_speed = 0.f;
//	}
//	
//}


/*弹速自适应*/
int8_t adapt_cnt = 0, low_cnt = 0,low_low_cnt = 0;

float last_measure_speed = 0.f;
float limit_Shoot_Speed = 25.f;

int8_t long_minus_step = -12;
int8_t short_minus_step = -4;
int8_t short_plus_step = 4;
int8_t long_plus_step = 10;

int8_t Adapt_k = 2;

static void My_Fric_Speed_Adapt(Launch_t* launch)//目标平均速度24.6f （24.7,24.5）
{
	static int8_t Adapt_Speed;
	float launch_speed_now;
	
	launch_speed_now = launch->judge.now_speed;
	
	if(board_cnt < 70)
	{
		/* 启动期：记录但不调节 */
		last_measure_speed = launch_speed_now;
		return;
	}
	
	/* 速度未更新或无效，跳过 */
	if((last_measure_speed == launch_speed_now) || (launch_speed_now <= 0))
	{
		return;
	}
	
	/* 目标24.6，允许范围24.5~24.7 */
	if(launch_speed_now >= (limit_Shoot_Speed - 0.2f))		// 24.8以上：严重过速
	{
		low_cnt = 0;
		adapt_cnt = 0;
		low_low_cnt = 0;
		Adapt_Speed = long_minus_step;
	}
	else if(launch_speed_now > (limit_Shoot_Speed - 0.4f))	// 24.6~24.8：轻微过速
	{
		low_cnt = 0;
		low_low_cnt = 0;
		adapt_cnt++;
		if(adapt_cnt >= 2)
		{
			Adapt_Speed = short_minus_step;
			adapt_cnt = 0;
		}
		else
		{
			Adapt_Speed = 0;  // 计数未满，暂不动作
		}
	}
	else if(launch_speed_now >= (limit_Shoot_Speed - 0.6f))	// 24.4~24.6：目标区间，不调节
	{
		adapt_cnt = 0;
		low_cnt = 0; 
		low_low_cnt = 0;
		Adapt_Speed = 0;
	}
	else if(launch_speed_now > (limit_Shoot_Speed - 0.8f))	// 24.2~24.4：轻微欠速
	{
		adapt_cnt = 0;
		low_low_cnt = 0;
		low_cnt++;
		if(low_cnt >= 3)
		{
			Adapt_Speed = short_plus_step;
			low_cnt = 0;  // 动作后清零，防止连续加
		}
		else
		{
			Adapt_Speed = 0;
		}
	}
	else													// 24.2以下：严重欠速
	{
		adapt_cnt = 0;
		low_cnt = 0;
		low_low_cnt ++;
		if(low_low_cnt >= 3)
		{
			Adapt_Speed = long_plus_step;
			low_low_cnt = 0;
		}
		else
		{
			Adapt_Speed = 0;
		}
	}
	
	/* 应用调节量 */
	if(Adapt_Speed != 0)
	{
		launch->info.fric_info.cfg_rx_info.base_cfg.normal_speed_target += Adapt_Speed * Adapt_k;
	}
	
	last_measure_speed = launch_speed_now;
}


int16_t heat_remain = 0;
void Muzzle_Heat_Detect(Launch_t* launch)
{
	heat_remain = launch->judge.muzzle_heat_max - launch->judge.muzzle_heat;
	if(launch->judge.muzzle_heat_max - launch->judge.muzzle_heat <= 50)
	{
		launch->base->info.rt_rx_info.flag_Info.run_limit_flag = 1;
		integral_to_zero(launch->assembly.group->motor[0]->ctrl->speed_ctrl);
		integral_to_zero(launch->assembly.group->motor[1]->ctrl->speed_ctrl);
//		integral_to_zero(launch->assembly.group->motor[2]->ctrl->speed_ctrl);
		
	}
	else{
		launch->base->info.rt_rx_info.flag_Info.run_limit_flag = 0;
		
		if(launch->base->info.rt_rx_info.flag_Info.fire_mode_flag == 1)
		{
			if(launch->judge.muzzle_heat_max - launch->judge.muzzle_heat <= 90 && launch->judge.muzzle_heat_max - launch->judge.muzzle_heat > 50)
	    {
		    launch->base->info.cfg_rx_info.base_cfg_info.reload_speed = DIAL_5_HZ_SPEED;
	    }
	    else{
	      launch->base->info.cfg_rx_info.base_cfg_info.reload_speed = DIAL_15_HZ_SPEED;
	    }
		}
	}

}

/**
  * @brief  摩擦轮PID计算
  * @note   拨盘在底盘，只给它发送目标值
  */
uint8_t fric_sleep[2];
void Fric_Pid_Cal(Launch_t* launch)
{
	uint8_t i = 0;
	int8_t k = 1;
	
	if(robot.state == LOST)
	{
		for(i = 0;i< FRICTION_LIST;i++)
	  {
			if(abs(launch->assembly.group->motor[i]->rx_info->encoder_speed) > 0 && fric_sleep[i] == 0)
			{
				launch->assembly.group->motor[i]->ctrl->speed_ctrl->target = 0;
		
		    launch->assembly.group->motor[i]->ctrl->speed_ctrl->measure = launch->assembly.group->motor[i]->rx_info->encoder_speed;
		    launch->assembly.group->motor[i]->ctrl->speed_ctrl->err = launch->assembly.group->motor[i]->ctrl->speed_ctrl->target 
		                                                                - launch->assembly.group->motor[i]->ctrl->speed_ctrl->measure;
		
		    single_pid_ctrl(launch->assembly.group->motor[i]->ctrl->speed_ctrl);
			  launch->assembly.group->motor[i]->tx_info->torque = launch->assembly.group->motor[i]->ctrl->speed_ctrl->out;
			}
			else{
				fric_sleep[i] = 1;
			  launch->assembly.tar.output = 0;
		
			  launch->assembly.group->motor[i]->tx_info->torque = launch->assembly.tar.output;
			}
		  
		}

	}
	
	else if(launch->base->cmd.fric_tx_cmd.work_state == STOP)           //卸力时不算PID
	{
			for(i = 0;i< FRICTION_LIST;i++)
	    {
				fric_sleep[i] = 0;
				
			  if(i == 0)
			  {
				  k = -1;
			  }
			  else{
				  k = 1;
			  }

				launch->assembly.group->motor[i]->ctrl->speed_ctrl->target = launch->assembly.tar.speed_target;
		
		    launch->assembly.group->motor[i]->ctrl->speed_ctrl->measure = launch->assembly.group->motor[i]->rx_info->encoder_speed;
		    launch->assembly.group->motor[i]->ctrl->speed_ctrl->err = launch->assembly.group->motor[i]->ctrl->speed_ctrl->target 
		                                                                - launch->assembly.group->motor[i]->ctrl->speed_ctrl->measure;
		
		    single_pid_ctrl(launch->assembly.group->motor[i]->ctrl->speed_ctrl);
			  launch->assembly.group->motor[i]->tx_info->torque = launch->assembly.group->motor[i]->ctrl->speed_ctrl->out;
				
	    }

//     for(i = 0;i< FRICTION_LIST;i++)
//	  {
//		  launch->assembly.tar.output = 0;
//			
//			launch->assembly.group->motor[i]->tx_info->torque = launch->assembly.tar.output;
//	  }

	}

	else if(launch->base->cmd.fric_tx_cmd.work_state == RUN)	
	{
			
		
		for(i = 0;i< FRICTION_LIST;i++)
	  {
			fric_sleep[i] = 0;
			
			if(i == 0)
			{
				k = -1;
			}
			else{
				k = 1;
			}
			
		  launch->assembly.group->motor[i]->ctrl->speed_ctrl->target = launch->assembly.tar.speed_target * k;
		
		  launch->assembly.group->motor[i]->ctrl->speed_ctrl->measure = launch->assembly.group->motor[i]->rx_info->encoder_speed;
		  launch->assembly.group->motor[i]->ctrl->speed_ctrl->err = launch->assembly.group->motor[i]->ctrl->speed_ctrl->target 
		                                                         - launch->assembly.group->motor[i]->ctrl->speed_ctrl->measure;
		
			if(launch->flag.fric_block_flag == 1)
		  {
			  launch->assembly.group->motor[i]->ctrl->speed_ctrl->out_max = 12000;
		  }		
			else{
			  launch->assembly.group->motor[i]->ctrl->speed_ctrl->out_max = 8000;
			}
			
		  single_pid_ctrl(launch->assembly.group->motor[i]->ctrl->speed_ctrl);
//		  launch->assembly.group->motor[i]->tx_info->torque = launch->assembly.group->motor[i]->ctrl->speed_ctrl->out;
			
			
		 launch->assembly.tar.output = launch->assembly.group->motor[i]->ctrl->speed_ctrl->out;
			
			launch->assembly.group->motor[i]->tx_info->torque = launch->assembly.tar.output;
			
			
	  }
	}
}

/**
  * @brief  发射机构扭矩与信号发送
  * @note   拨盘在底盘，只给它发送目标值
  */
uint8_t my_tx_buff[8];
uint8_t flas = 0;
void Launch_Send(Launch_t* launch)
{
	launch->assembly.group->group_set_torque(launch->assembly.group);
   
//	My_Torque_to_Raw_Current(&Fric_R_Motor);
//	My_Torque_to_Raw_Current(&Fric_L_Motor);
//	
//  my_tx_buff[0] = (uint8_t)(Fric_R_Motor.tx_info->torque_current_raw >> 8);
//	my_tx_buff[1] = (uint8_t)(Fric_R_Motor.tx_info->torque_current_raw);
//	
//	my_tx_buff[2] = (uint8_t)(Fric_L_Motor.tx_info->torque_current_raw >> 8);
//  my_tx_buff[3] = (uint8_t)(Fric_L_Motor.tx_info->torque_current_raw);

//	my_tx_buff[4] = 0;
//  my_tx_buff[5] = 0;

//	my_tx_buff[6] = 0;
//  my_tx_buff[7] = 0;
//		
//	CAN1_SendData(0x200, my_tx_buff);
//	
//	if(my_tx_buff[3] == 0)
//	{
//		flas = 1;
//	}
//	else{
//   flas = 0;	
//	}
//		
//	memset(my_tx_buff, 0, 8);
	 
}

/**
  * @brief   发射机构总控
  */

void Launch_Work(Launch_t* launch)
{
	Vision_Tx_Update(launch);
	Fric_Block_Check(launch);           
	Fric_State_Check(launch);     
  Launch_Data_Update(launch);	
	Launch_Flag_Update(launch);
//	My_Fric_Speed_Adapt(launch);
	Shoot_Base_Work(launch->base);
	Fric_Pid_Cal(launch);
  Launch_Send(launch);
	
}
