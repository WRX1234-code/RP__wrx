#include "shoot_base.h"
#include <stdlib.h>

Shoot_t shoot;

/*--------------------------------对内API定义----------------------------------*/

/**
 * @brief   计算角度和
 * @note    用于相对角度计算，和绝对角度分析
 */
static void Angle_Sum_Calculate(Shoot_t* shoot)
{
	static DIAL_ANGLE_DATA_TYPE  last_angle;
	static DIAL_ANGLE_DATA_TYPE  now_angle;
	DIAL_ANGLE_SUM_DATA_TYPE  angle_err;
	
	now_angle = shoot->info.rt_rx_info.dial_info.angle;
	
	if(last_angle == 0 && now_angle == 0)
	{
		angle_err = 0;
	}
	else
  {
		angle_err = now_angle - last_angle;
	}
	
	if(angle_err > (DIAL_ANGLE_MAX - DIAL_ANGLE_MIN) / 2)
	{
		shoot->misc.angle_sum += (DIAL_ANGLE_MIN - DIAL_ANGLE_MAX + angle_err);
	}
	else if(angle_err <= (DIAL_ANGLE_MIN - DIAL_ANGLE_MAX) / 2)
	{
		shoot->misc.angle_sum += (DIAL_ANGLE_MAX - DIAL_ANGLE_MIN + angle_err);
	}
	else
	{
		shoot->misc.angle_sum += angle_err; 
	}
  	
	last_angle = now_angle;
}

/**
 * @brief   限制绝对角度
 * @note    只用于绝对角度，避免超出绝对角度的取值范围
 */
static void Absolute_Angle_Limit(DIAL_ANGLE_DATA_TYPE* unlimited_angle)
{
	if(DIAL_IS_ANSOLUTE_ANGLE)
	{
		DIAL_ANGLE_DATA_TYPE angle = *unlimited_angle; 
		if(angle > DIAL_ANGLE_MAX)
		{
			angle -= DIAL_ANGLE_MAX - DIAL_ANGLE_MIN;
		}
		else if(angle < DIAL_ANGLE_MIN)
		{
			angle += DIAL_ANGLE_MAX - DIAL_ANGLE_MIN;
		}
		
		*unlimited_angle = angle;
		
	}
}

/**
 * @brief   上弹和退弹状态下角度环的目标角度转化
 */
static void Angle_Target_Switch(Shoot_t* shoot)
{
	int8_t k;
	if(shoot->cmd.dial_tx_cmd.work_state == RELOAD)
	{
		k = 1;
	}
	else if(shoot->cmd.dial_tx_cmd.work_state == RECOIL)
	{
		k = -1;
	}
	
	if(DIAL_IS_ANSOLUTE_ANGLE)
	{
		shoot->cmd.dial_tx_cmd.angle_target += k * shoot->info.cfg_rx_info.base_cfg_info.oneshot_angle;
	  Absolute_Angle_Limit(&shoot->cmd.dial_tx_cmd.angle_target);
  }
	else
  {
	  shoot->cmd.dial_tx_cmd.angle_sum_target += k * shoot->info.cfg_rx_info.base_cfg_info.oneshot_angle;
  }
}


/*---------------------------------对外API定义-------------------------------------*/

/**
 * @brief   初始化发射机构状态与默认参数
 * @note    仅赋值状态，不会启动电机；配置项在外部 json 加载
 * @warning 必须在所有电机上电前调用
 */
void Shoot_Init(Shoot_t* shoot)
{
	//初始化工作状态
	shoot->work_state = LOCKED;
	//初始化发射机构模式
	shoot->mode = CEASEFIRE;
	//初始化命令
	shoot->cmd.dial_tx_cmd.work_state = SLEEP;
	shoot->cmd.fric_tx_cmd.work_state = STOP;
	//初始化标志位
	shoot->info.rt_rx_info.flag_Info.is_sleep_flag = 1;
	shoot->info.rt_rx_info.flag_Info.is_mtr_offline_flag = 0;
	shoot->info.rt_rx_info.flag_Info.elec_level_flag = 0;
	
	shoot->flag.dial_block_flag = 0;
	shoot->flag.init_flag = 0;
	shoot->flag.reset_speed_flag = 0;
	
	/*在此处配置电机结构体config，不得有漏配置
	
	//拨盘基本配置
	shoot->shoot_info.cfg_rx_info.base_cfg_info.repeat_shot_mode =;
	shoot->shoot_info.cfg_rx_info.base_cfg_info.repeat_shot_period =;
	shoot->shoot_info.cfg_rx_info.base_cfg_info.oneshot_angle =;
	shoot->shoot_info.cfg_rx_info.base_cfg_info.reload_speed =;
	shoot->shoot_info.cfg_rx_info.base_cfg_info.reset_adjust_angle =;
	shoot->shoot_info.cfg_rx_info.base_cfg_info.reset_speed =;
	shoot->shoot->shoot_info.cfg_rx_info.base_cfg_info.reset_angle =;
	shoot->shoot_info.cfg_rx_info.base_cfg_info.reset_work_time_max =;
	shoot->shoot_info.cfg_rx_info.base_cfg_info.speed_stop_mode =;
	shoot->shoot_info.cfg_rx_info.base_cfg_info.state_work_time_max =;
	
	//拨盘复位堵转配置
	
	shoot->shoot_info.cfg_rx_info.reset_block_cfg_info.angle_sum_err_integral_max =;
	shoot->shoot_info.cfg_rx_info.reset_block_cfg_info.block_judge_type =;
	
	shoot->shoot_info.cfg_rx_info.reset_block_cfg_info.block_time_max =;
	shoot->shoot_info.cfg_rx_info.reset_block_cfg_info.current_min =;
	shoot->shoot_info.cfg_rx_info.reset_block_cfg_info.integral_value =;
	shoot->shoot_info.cfg_rx_info.reset_block_cfg_info.speed_max =;

  //拨盘速度环堵转配置
	
	shoot->shoot_info.cfg_rx_info.reload_speed_block_cfg_info.angle_sum_err_integral_max =;
	shoot->shoot_info.cfg_rx_info.reload_speed_block_cfg_info.block_judge_type =;
	
	shoot->shoot_info.cfg_rx_info.reload_speed_block_cfg_info.block_time_max =;
	shoot->shoot_info.cfg_rx_info.reload_speed_block_cfg_info.current_min =;
	shoot->shoot_info.cfg_rx_info.reload_speed_block_cfg_info.integral_value =;
	shoot->shoot_info.cfg_rx_info.reload_speed_block_cfg_info.speed_max = ;
	
	//拨盘角度环堵转配置
	
	shoot->shoot_info.cfg_rx_info.reload_angle_block_cfg_info.angle_sum_err_integral_max =;
	shoot->shoot_info.cfg_rx_info.reload_angle_block_cfg_info.block_judge_type =;
	
	shoot->shoot_info.cfg_rx_info.reload_angle_block_cfg_info.block_time_max =;
	shoot->shoot_info.cfg_rx_info.reload_angle_block_cfg_info.current_min =;
	shoot->shoot_info.cfg_rx_info.reload_angle_block_cfg_info.integral_value =;
	shoot->shoot_info.cfg_rx_info.reload_angle_block_cfg_info.speed_max =;
	
	*/

}


/**
 * @brief   更新拨盘工作状态
 * @note    内部使用 static 变量保存节拍计数，不可重入
 */
void Shoot_Work_State_Update(Shoot_t* shoot)
{
	if(shoot->info.rt_rx_info.flag_Info.is_sleep_flag == 1 &&shoot->flag.init_flag == 0)
	{
		shoot->work_state = LOCKED;                                   //关保险状态更新
		
		shoot->mode = CEASEFIRE;
		
		shoot->cmd.dial_tx_cmd.work_state = SLEEP;
		shoot->cmd.fric_tx_cmd.work_state = STOP;
		
	}

	else if(shoot->info.rt_rx_info.flag_Info.is_sleep_flag == 0 && shoot->flag.init_flag == 0 
		                                                          && shoot->cmd.vision_tx_cmd.is_ready_flag == 1)
	{
		shoot->work_state = INITING;                                  //初始化状态更新
		shoot->cmd.fric_tx_cmd.work_state = RUN;
		
	}
	
	else if(shoot->info.rt_rx_info.flag_Info.is_sleep_flag == 0 && shoot->flag.init_flag == 1)
	{
		shoot->work_state = UNLOCK;                                   //开保险状态更新
		
    shoot->cmd.fric_tx_cmd.work_state = RUN;                      //摩擦轮开转
		
  }
}

/**
* @brief   发射机构模式实时更新
* @note    fire_mode_flag外部文件更新数值     
 */
void Shoot_Mode_Update(Shoot_t* shoot)
{
	static uint8_t last_shoot_mode;
	static uint8_t last_dial_mode;
	
	if(shoot->info.rt_rx_info.flag_Info.fire_mode_flag == 0)
	{
		shoot->mode = SINGLE_SHOT;
		shoot->cmd.dial_tx_cmd.mode = DIAL_ANGLE;
		
		//连发命令取消时拨盘原地停下的前提，连发速度环切单发角度环需要补偿小角度来保持最佳单发发射状态
		if(last_shoot_mode == REPEAT_SHOT && last_dial_mode == DIAL_SPEED && shoot->info.cfg_rx_info.base_cfg_info.speed_stop_mode == STAND)
		{
			shoot->cmd.dial_tx_cmd.angle_sum_target -= shoot->misc.switch_adjust_angle;
			shoot->cmd.vision_tx_cmd.is_ready_flag = 0;
		}
	}
	else if(shoot->info.rt_rx_info.flag_Info.fire_mode_flag == 1)
	{
		shoot->mode = REPEAT_SHOT;
		
		if(shoot->info.cfg_rx_info.base_cfg_info.repeat_shot_mode == DIAL_ANGLE)              //角度环连发
		{
			shoot->cmd.dial_tx_cmd.mode = DIAL_ANGLE;
			//连发命令取消时拨盘原地停下的前提，连发速度环切连发角度环也需要补偿角度
			if(last_dial_mode == DIAL_SPEED && shoot->info.cfg_rx_info.base_cfg_info.speed_stop_mode == STAND)
			{
				shoot->cmd.dial_tx_cmd.angle_sum_target -= shoot->misc.switch_adjust_angle;
			}
		}
		else if(shoot->info.cfg_rx_info.base_cfg_info.repeat_shot_mode == DIAL_SPEED)          //速度环连发
		{
			shoot->cmd.dial_tx_cmd.mode = DIAL_SPEED;
			shoot->cmd.dial_tx_cmd.angle_sum_target = shoot->misc.angle_sum;          //实时更新角度和目标值
		}
	}
	else
	{
		shoot->mode = CEASEFIRE;                                           //停火模式
	}
	
	last_shoot_mode = shoot->mode;
	last_dial_mode = shoot->cmd.dial_tx_cmd.mode;
}

/**
 * @brief   检测拨盘是否堵转
 * @param   info   - 拨盘实时数据
 * @param   config - 堵转判断阈值配置
 * @retval  1 - 堵转；0 - 正常
 * @note    阻塞时间单位：1 ms 调用周期
 */
uint8_t Dial_Block_Check(Dial_Rt_Rx_Info_t* rt_info,Shoot_Misc_t* misc,Dial_Block_Cfg_Rx_Info_t* cfg_info,Dial_Tx_Cmd_t* cmd)
{
	uint8_t flag = 0;
	if(cfg_info->block_judge_type == 0)   //堵转判断方法1
	{
		if(abs(rt_info->speed) < cfg_info->speed_max && abs(rt_info->current) > cfg_info->current_min)
    {
		  if(cfg_info->block_time >= cfg_info->block_time_max)
		  {
		  	flag = 1;
				cfg_info->block_time = 0;
		  }
		  else
		  {
		    cfg_info->block_time ++;
	    } 
	  }
	  else
	  {
		   cfg_info->block_time = 0;
	  }
	}
	
	else if(cfg_info->block_judge_type == 1 && cmd->mode == DIAL_ANGLE)    //堵转判断方法2，只适用于角度环
	{
		
		#if DIAL_IS_ANSOLUTE_ANGLE
		  if(cmd->angle_target - rt_info->angle > shoot.info.cfg_rx_info.base_cfg_info.stop_angle_err_max)
		 	{
		    if((cmd->angle_target - rt_info->angle) / rt_info->current > 0)
		    {
			    cfg_info->angle_err_integral += cfg_info->integral_value * (cmd->angle_target - rt_info->angle);
		    }
		    else if((cmd->angle_target - rt_info->angle) / rt_info->current < 0)
		    {
		  	  if(cmd->angle_target >= rt_info->angle)
		  	  {
			  	  cfg_info->angle_err_integral += cfg_info->integral_value * (cmd->angle_target - rt_info->angle + DIAL_ANGLE_MIN - DIAL_ANGLE_MAX);
			    }                                                                                                 
			    else if(cmd->angle_target < rt_info->angle)
			    {
				    cfg_info->angle_err_integral += cfg_info->integral_value * (DIAL_ANGLE_MIN- DIAL_ANGLE_MAX + rt_info->angle - cmd->angle_target);
			    }
	    	}
		    if(cfg_info->angle_err_integral >= cfg_info->angle_err_integral_max) 
		    {
		  	  flag = 1;
			    cfg_info->angle_err_integral = 0;
		    }
		  }
		  else
		  {
			  cfg_info->angle_err_integral = 0;
			}	
			
			
		#else
		  if(cmd->angle_sum_target - misc->angle_sum > shoot.info.cfg_rx_info.base_cfg_info.stop_angle_err_max)
			{
				cfg_info->angle_sum_err_integral += cfg_info->integral_value * (cmd->angle_sum_target - misc->angle_sum);
		    if(abs(cfg_info->angle_sum_err_integral) >= cfg_info->angle_sum_err_integral_max)
		    {
			    flag = 1;
		  	  cfg_info->angle_sum_err_integral = 0;
		    }
			}
			else
      {
				cfg_info->angle_sum_err_integral = 0;
			}
			
		#endif
	}
	
	return flag;
}

///**
// * @brief   检测摩擦轮是否堵转
// * @param   info   - 拨盘实时数据
// * @param   config - 堵转判断阈值配置
// * @retval  1 - 堵转；0 - 正常
// * @note    阻塞时间单位：1 ms 调用周期
// */
//uint8_t Fric_Block_Check(Shoot_t* shoot)
//{
//	uint8_t flag = 0;
//	for(uint8_t i = 0;i < FRIC_LIST;i ++)
//	{
//		if(abs(shoot->fric.info[i].speed) < shoot->fric.config.block_config.speed_max
//		&& abs(shoot->fric.info[i].current) > shoot->fric.config.block_config.current_min)
//	  {
//			if(shoot->fric.config.block_config.block_time[i] >= shoot->fric.config.block_config.block_time_max)
//			{
//				flag = 1;
//				shoot->shoot_flag.fric_block_flag = 1;
//			}
//			else
//			{
//		  	shoot->fric.config.block_config.block_time[i] ++;
//			}
//	  }
//		else
//		{
//			shoot->fric.config.block_config.block_time[i] = 0;
//		}
//	}
//	
//	return flag;
//}

/**
 * @brief   拨盘实时状态更新
 * @note    elec_level_flag外部文件更新
 */
void Dial_Work_State_Update(Shoot_t* shoot)
{
	static uint8_t last_elec_level_flag;                     //保存上一个电平标志位，用于检测电平变化
	static uint8_t work_time;                                //记录工作时间，想看状态变化可以将其提升至全局变量
	static uint16_t now_tick;                                //保存当前tick值
	static uint16_t last_tick;                               //保存上一次tick值
	 
	switch (shoot->cmd.dial_tx_cmd.work_state)
	{
		case SLEEP:                                              //睡眠模式更新，只卸力
			shoot->cmd.dial_tx_cmd.current_target = 0;
		  shoot->cmd.fric_tx_cmd.work_state = STOP;
		
		  if(shoot->info.rt_rx_info.flag_Info.is_sleep_flag == 0 && shoot->flag.init_flag == 0 
		                                                          && shoot->cmd.vision_tx_cmd.is_ready_flag == 1)
			{
		     shoot->cmd.dial_tx_cmd.work_state = RESETING;                 //拨盘进入复位状态
		
		     if(DIAL_IS_ANSOLUTE_ANGLE)
		     { 
		       shoot->cmd.dial_tx_cmd.mode = DIAL_ANGLE;
			     shoot->cmd.dial_tx_cmd.angle_target = shoot->info.cfg_rx_info.base_cfg_info.reset_angle;
		     }
		     else
		     {
			     shoot->cmd.dial_tx_cmd.mode = DIAL_SPEED;
					 //宏定义用于变换速度方向，方向取决于拨盘正转使弹丸触碰限位还是反转触碰，正转是碰到枪管限位	 
		       shoot->cmd.dial_tx_cmd.speed_target = -DIAL_MEC_LIMIT * shoot->info.cfg_rx_info.base_cfg_info.reset_speed;  
	       }
			} 
			
		  break;
		
		case RESETING:                                            //复位状态更新
			work_time ++;
		  
		  if(DIAL_IS_ANSOLUTE_ANGLE == 0)
			{
				if(Dial_Block_Check(&shoot->info.rt_rx_info.dial_info,NULL,
				                  &shoot->info.cfg_rx_info.reset_speed_block_cfg_info,&shoot->cmd.dial_tx_cmd) == 1)   //拨盘堵转返回 1
			  {
			  	//对拨盘回转调整角度限幅，防止过大
				  if(shoot->info.cfg_rx_info.base_cfg_info.reset_adjust_angle > shoot->info.cfg_rx_info.base_cfg_info.oneshot_angle)
				  {
				   	shoot->info.cfg_rx_info.base_cfg_info.reset_adjust_angle = shoot->info.cfg_rx_info.base_cfg_info.oneshot_angle;
				  }
				  else if(shoot->info.cfg_rx_info.base_cfg_info.reset_adjust_angle < -shoot->info.cfg_rx_info.base_cfg_info.oneshot_angle)
				  {
				   	shoot->info.cfg_rx_info.base_cfg_info.reset_adjust_angle = -shoot->info.cfg_rx_info.base_cfg_info.oneshot_angle;
				  }
				
		      shoot->cmd.dial_tx_cmd.angle_sum_target += DIAL_MEC_LIMIT * shoot->info.cfg_rx_info.base_cfg_info.reset_adjust_angle;
		
				  shoot->cmd.dial_tx_cmd.mode = DIAL_ANGLE;
				  shoot->flag.reset_speed_flag = 1;
        
          work_time = 0;		
				}	
				
			  //超时退出，进入等待模式
			  else if(work_time >= shoot->info.cfg_rx_info.base_cfg_info.reset_speed_work_time_max)
			  {
				  shoot->cmd.dial_tx_cmd.work_state = WAITING;
		      shoot->cmd.dial_tx_cmd.mode = DIAL_ANGLE;
				  shoot->flag.init_flag = 1;
				  //记录拨盘的起始角度和，用于后续计算补偿角
				  shoot->misc.angle_sum_start = shoot->misc.angle_sum;     
        }  			
				
			}
			
			else if(DIAL_IS_ANSOLUTE_ANGLE == 1 || (DIAL_IS_ANSOLUTE_ANGLE == 0 && shoot->flag.reset_speed_flag == 1))
			{
				if(Dial_Block_Check(&shoot->info.rt_rx_info.dial_info,&shoot->misc,
				                  &shoot->info.cfg_rx_info.angle_block_cfg_info,&shoot->cmd.dial_tx_cmd) == 1)   
			  {
			  	shoot->cmd.dial_tx_cmd.work_state = WAITING;
				  shoot->flag.init_flag = 1;
				  //记录拨盘的起始角度和，用于后续计算补偿角
				  shoot->misc.angle_sum_start = shoot->misc.angle_sum;     
			  }
			  else if(work_time >= shoot->info.cfg_rx_info.base_cfg_info.reset_angle_work_time_max)
			  {
				  shoot->cmd.dial_tx_cmd.work_state = WAITING;
				  shoot->flag.init_flag = 1;
				  //记录拨盘的起始角度和，用于后续计算补偿角
				  shoot->misc.angle_sum_start = shoot->misc.angle_sum;     
			  }
			}
		  
			//初始化完成，切换进等待模式
			if(shoot->flag.init_flag == 0 && ((abs(shoot->cmd.dial_tx_cmd.angle_sum_target - shoot->misc.angle_sum) 
			                              <= shoot->info.cfg_rx_info.base_cfg_info.stop_angle_err_max)
			                              || DIAL_ANSOLUTE_ANGLE_STOP))
			{                                
				shoot->cmd.dial_tx_cmd.work_state = WAITING;
				shoot->cmd.dial_tx_cmd.mode = DIAL_ANGLE;
				shoot->flag.init_flag = 1;
				
				//记录拨盘的起始角度和，用于后续计算补偿角
				shoot->misc.angle_sum_start = shoot->misc.angle_sum;  				
   
			}
		
		  break;
			
		case WAITING:                                            //等待模式更新，角度环
		  shoot->cmd.dial_tx_cmd.mode = DIAL_ANGLE;
		  work_time = 0;
		
	    //只要INITING外部更新，可多次切换成复位模式
		  if(shoot->work_state == INITING)                
			{
				shoot->cmd.dial_tx_cmd.work_state = RESETING;
			}
			//需要同时符合可以立即开火，开火操作和当前开火模式对应的开火操作相同，才能补弹
			if(shoot->cmd.vision_tx_cmd.is_ready_flag == 1)        //可以立即开火
			{
				if(shoot->work_state == SINGLE_SHOT && last_elec_level_flag == 0 && shoot->info.rt_rx_info.flag_Info.elec_level_flag == 1)  //上升沿触发 
			  {
				  shoot->cmd.dial_tx_cmd.work_state = RELOAD;        //进入补弹模式
				  Angle_Target_Switch(shoot);
					
					shoot->cmd.vision_tx_cmd.is_ready_flag = 0;        //此时不能立即开火，为了防止补弹未完成时中有开火操作而导致再次补弹
			  }
			  else if(shoot->work_state == REPEAT_SHOT && last_elec_level_flag == 1 && shoot->info.rt_rx_info.flag_Info.elec_level_flag == 1)  //高电平触发
        {
			    if(shoot->info.cfg_rx_info.base_cfg_info.repeat_shot_mode == DIAL_ANGLE)
				  {
						shoot->cmd.dial_tx_cmd.work_state = RELOAD;
				  	Angle_Target_Switch(shoot);
				  	shoot->cmd.dial_tx_cmd.mode = DIAL_ANGLE;
						shoot->cmd.vision_tx_cmd.is_ready_flag = 0;      //此时不能立即开火，为了防止补弹未完成时，或补弹完成但周期未到时，有开火操作而导致再次补弹
					
					  last_tick = HAL_GetTick();
				  }
				  else if(shoot->info.cfg_rx_info.base_cfg_info.repeat_shot_mode == DIAL_SPEED)
				  {
						shoot->cmd.dial_tx_cmd.work_state = RELOAD;
				  	shoot->cmd.dial_tx_cmd.speed_target = shoot->info.cfg_rx_info.base_cfg_info.reload_speed;
					  shoot->cmd.dial_tx_cmd.mode = DIAL_SPEED;
				  }
		  	}
			}
			
		  break;
		
		
		case RELOAD:                             //补弹模式更新
			
		  switch (shoot->cmd.dial_tx_cmd.mode)
		  {
		  	case DIAL_ANGLE:
			  	work_time ++;
          //角度环连发有发弹周期
          if(shoot->mode == REPEAT_SHOT)
				  {
					  now_tick = HAL_GetTick();
			  	  //判断连发周期是否到达在决定是否打弹
				    if(now_tick - last_tick >= shoot->info.cfg_rx_info.base_cfg_info.repeat_shot_period)
				    {
				  	  shoot->cmd.dial_tx_cmd.angle_sum_target += shoot->info.cfg_rx_info.base_cfg_info.oneshot_angle;
					    work_time ++;
					    last_tick = now_tick;                              //实时更新tick
				    }
				    //拨盘在周期内完成角度环并停下
				    else if(now_tick - last_tick < shoot->info.cfg_rx_info.base_cfg_info.repeat_shot_period 
				 	                               && abs(shoot->cmd.dial_tx_cmd.angle_sum_target - shoot->misc.angle_sum) 
						                             <=shoot->info.cfg_rx_info.base_cfg_info.stop_angle_err_max)
				    {
					    work_time = 0;
							
							shoot->info.cfg_rx_info.angle_block_cfg_info.block_time = 0;
              shoot->info.cfg_rx_info.angle_block_cfg_info.angle_sum_err_integral = 0;			
            }
			  	}					
				
				   //实时更新拨弹进度
				  shoot->cmd.vision_tx_cmd.reload_sche = (float)(shoot->cmd.dial_tx_cmd.angle_sum_target - shoot->misc.angle_sum) 
		                                   / (float)shoot->info.cfg_rx_info.base_cfg_info.oneshot_angle;	
				
				  //堵转处理切退弹模式
				  if(Dial_Block_Check(&shoot->info.rt_rx_info.dial_info,&shoot->misc,&shoot->info.cfg_rx_info.angle_block_cfg_info,&shoot->cmd.dial_tx_cmd) == 1)
			    {
			  	  shoot->cmd.dial_tx_cmd.work_state = RECOIL;
				    Angle_Target_Switch(shoot);
				    shoot->flag.dial_block_flag = 1;
				  	//清零工作时间，防止后面误入分支
				    work_time = 0;  
						
					  shoot->info.rt_rx_info.flag_Info.elec_level_flag = 0;
					  shoot->cmd.vision_tx_cmd.is_ready_flag = 0;
			    }
				  //超时退出
			    else if(work_time >= shoot->info.cfg_rx_info.base_cfg_info.state_work_time_max)
			    {
				    shoot->cmd.dial_tx_cmd.work_state = WAITING;
					  shoot->cmd.dial_tx_cmd.mode = DIAL_ANGLE;
					  work_time = 0;
						
				  	shoot->info.rt_rx_info.flag_Info.elec_level_flag = 0;
				  	shoot->cmd.vision_tx_cmd.is_ready_flag = 1;
						
						shoot->info.cfg_rx_info.angle_block_cfg_info.block_time = 0;
            shoot->info.cfg_rx_info.angle_block_cfg_info.angle_sum_err_integral = 0;			
			    }
				  //拨盘完成角度环，停下来才切换等待模式
				  else if(shoot->misc.angle_sum <= shoot->info.cfg_rx_info.base_cfg_info.stop_angle_err_max)
				  {
					  shoot->cmd.dial_tx_cmd.work_state = WAITING;
					  shoot->cmd.dial_tx_cmd.mode = DIAL_ANGLE;
					  work_time = 0;
					  shoot->info.rt_rx_info.flag_Info.elec_level_flag = 0;
					  shoot->cmd.vision_tx_cmd.is_ready_flag = 1;
						
						shoot->info.cfg_rx_info.angle_block_cfg_info.block_time = 0;
            shoot->info.cfg_rx_info.angle_block_cfg_info.angle_sum_err_integral = 0;			
				  }
		    
	        break;
				
		    case DIAL_SPEED:
				
			    if(shoot->info.rt_rx_info.flag_Info.elec_level_flag == 1)
				  {
				  	//计算补偿角，用于后续速度环切角度环时拨盘回转以补偿超出角度环目标值集合的角度
				    shoot->misc.switch_adjust_angle = (shoot->misc.angle_sum - shoot->misc.angle_sum_start)
				                                                         % shoot->info.cfg_rx_info.base_cfg_info.oneshot_angle;
						
				    //速度环与角度环计算方式稍微不同
				    shoot->cmd.vision_tx_cmd.reload_sche = (float)(shoot->info.cfg_rx_info.base_cfg_info.oneshot_angle - shoot->misc.switch_adjust_angle) 
		                                      / (float)shoot->info.cfg_rx_info.base_cfg_info.oneshot_angle;	
						
				    //实时更新角度和目标值
				    shoot->cmd.dial_tx_cmd.angle_sum_target = shoot->misc.angle_sum;
				    //堵转处理
			      if(Dial_Block_Check(&shoot->info.rt_rx_info.dial_info,NULL,&shoot->info.cfg_rx_info.speed_block_cfg_info,&shoot->cmd.dial_tx_cmd) == 1)
			      {
			        shoot->cmd.dial_tx_cmd.work_state = RECOIL;
				      shoot->cmd.dial_tx_cmd.mode = DIAL_ANGLE;
				      Angle_Target_Switch(shoot);                              //堵转也要调整一弹丸角度，后面会补回来
				      shoot->flag.dial_block_flag = 1;
				      shoot->info.rt_rx_info.flag_Info.elec_level_flag = 0;
				      shoot->cmd.vision_tx_cmd.is_ready_flag = 0;
				      work_time = 0;
			      }
				  }
				  else if(shoot->info.rt_rx_info.flag_Info.elec_level_flag == 0)       //连发开火停止
				  {
					  shoot->cmd.dial_tx_cmd.work_state = WAITING;
					  shoot->cmd.dial_tx_cmd.mode = DIAL_ANGLE;
					  shoot->cmd.vision_tx_cmd.is_ready_flag = 1;
		        work_time = 0;
						//速度环连发停止时有四种归位模式
						switch (shoot->info.cfg_rx_info.base_cfg_info.speed_stop_mode)
						{
						  case STAND:                //原地不动，此时目标值已经达到
							 break;
							
							case FORWARD:              //往前转动，达到前面的角度环集合目标值
								shoot->cmd.dial_tx_cmd.angle_sum_target +=(shoot->info.cfg_rx_info.base_cfg_info.oneshot_angle - shoot->misc.switch_adjust_angle);
							  break;
							
							case BACKWARD:             //往后转动，达到后面的角度环集合目标值
								shoot->cmd.dial_tx_cmd.angle_sum_target -= shoot->misc.switch_adjust_angle;
							  break;
							
							case ROUNDING_UP:          //四舍五入，距离哪边近就去哪边
								if(2 * shoot->misc.switch_adjust_angle > shoot->info.cfg_rx_info.base_cfg_info.oneshot_angle)
								{
									shoot->cmd.dial_tx_cmd.angle_sum_target +=(shoot->info.cfg_rx_info.base_cfg_info.oneshot_angle - shoot->misc.switch_adjust_angle);
								}
								else if(2 * shoot->misc.switch_adjust_angle <= shoot->info.cfg_rx_info.base_cfg_info.oneshot_angle)
								{
									shoot->cmd.dial_tx_cmd.angle_sum_target -= shoot->misc.switch_adjust_angle;
								}
								break;
						}
				  }
			    break;
			 }
				  
		   break;	
				 
		case RECOIL:                                                           //堵转状态更新
			//清零堵转参考值
			shoot->info.cfg_rx_info.angle_block_cfg_info.block_time = 0;         
      shoot->info.cfg_rx_info.angle_block_cfg_info.angle_sum_err_integral = 0;			
		  shoot->info.cfg_rx_info.speed_block_cfg_info.block_time = 0;
		  //成功退弹
			if(shoot->misc.angle_sum <= shoot->cmd.dial_tx_cmd.angle_sum_target)
			{
				shoot->cmd.dial_tx_cmd.work_state = WAITING;
				shoot->cmd.dial_tx_cmd.mode = DIAL_ANGLE;
				work_time = 0; 
				
			}
			//超时退出
		  else if(work_time >= shoot->info.cfg_rx_info.base_cfg_info.state_work_time_max)
			{
				shoot->cmd.dial_tx_cmd.work_state = WAITING;
				shoot->cmd.dial_tx_cmd.mode = DIAL_ANGLE;
				work_time = 0; 
			}
			else
			{
				work_time ++;
			}

		  break;
	}
	
}

///**
// * @brief   检测摩擦轮是否超速或低速，高温
// */
//void Fric_State_Check(Shoot_t* shoot)
//{
//	//这个 k 在宏定义中有指出
//	int8_t k = 1;
//	//宏定义顶替一段可能较长的，用于通过改变 k 为 1和 -1 的逻辑，从而改变速度方向
//	FRIC_SPEED_DATA_DIRECTION_MENAGE
//	
//	uint8_t i;
//	for(i = 0;i < FRIC_LIST;i ++)                //遍历求差，便于debug
//	{
//		shoot->fric.check.speed_err[i] = abs(shoot->fric.info[i].speed - k*shoot->fric.config.base_config.normal_speed_target);
//		shoot->fric.check.temp_err[i] = shoot->fric.info[i].temperature - shoot->fric.config.base_config.temp_max;
//	}
//	//枚举选出对应的摩擦轮分别查看情况
//	#if FRIC_NUM == 6             //六摩
//	
//	if(shoot->fric.check.speed_err[FRIC_B_UP] < shoot->fric.config.base_config.speed_err_max
//		&& shoot->fric.check.speed_err[FRIC_B_R] < shoot->fric.config.base_config.speed_err_max
//    && shoot->fric.check.speed_err[FRIC_B_L] < shoot->fric.config.base_config.speed_err_max
//    && shoot->fric.check.speed_err[FRIC_F_UP] < shoot->fric.config.base_config.speed_err_max
//    && shoot->fric.check.speed_err[FRIC_F_R] < shoot->fric.config.base_config.speed_err_max
//    && shoot->fric.check.speed_err[FRIC_F_L] < shoot->fric.config.base_config.speed_err_max	)

//  #elif FRIC_NUM == 3	           //三摩
//	if(shoot->fric.check.speed_err[FRIC_UP] < shoot->fric.config.base_config.speed_err_max
//		&& shoot->fric.check.speed_err[FRIC_R] < shoot->fric.config.base_config.speed_err_max
//	  && shoot->fric.check.speed_err[FRIC_L] < shoot->fric.config.base_config.speed_err_max)

//  #elif FRIC_NUM == 2	           //二摩
//	if(shoot->fric.check.speed_err[FRIC_R] < shoot->fric.config.base_config.speed_err_max
//		&& shoot->fric.check.speed_err[FRIC_L] < shoot->fric.config.base_config.speed_err_max)
//	#endif
//	{
//		shoot->shoot_flag.fric_normal_speed_flag = 1;
//	}
//	else 
//	{
//		shoot->shoot_flag.fric_normal_speed_flag = 0;
//	}	
//	
//	
//	#if FRIC_NUM == 6                //六摩
//	
//	if(shoot->fric.check.temp_err[FRIC_B_UP] < shoot->fric.config.base_config.temp_err_max
//		&& shoot->fric.check.temp_err[FRIC_B_R] < shoot->fric.config.base_config.temp_err_max
//    && shoot->fric.check.temp_err[FRIC_B_L] < shoot->fric.config.base_config.temp_err_max
//    && shoot->fric.check.temp_err[FRIC_F_UP] < shoot->fric.config.base_config.temp_err_max
//    && shoot->fric.check.temp_err[FRIC_F_R] < shoot->fric.config.base_config.temp_err_max
//    && shoot->fric.check.temp_err[FRIC_F_L] < shoot->fric.config.base_config.temp_err_max	)

//  #elif FRIC_NUM == 3	             //三摩
//	if(shoot->fric.check.temp_err[FRIC_UP] < shoot->fric.config.base_config.temp_err_max
//		&& shoot->fric.check.temp_err[FRIC_R] < shoot->fric.config.base_config.temp_err_max
//	  && shoot->fric.check.temp_err[FRIC_L] < shoot->fric.config.base_config.temp_err_max)

//  #elif FRIC_NUM == 2	             //二摩
//	if(shoot->fric.check.temp_err[FRIC_R] < shoot->fric.config.base_config.temp_err_max
//		&& shoot->fric.check.temp_err[FRIC_L] < shoot->fric.config.base_config.temp_err_max)
//	#endif
//	{
//		shoot->shoot_flag.fric_high_temp_flag = 0;
//	}
//	else 
//	{
//		shoot->shoot_flag.fric_high_temp_flag = 1;
//	}	
//	
//}


///**
// * @brief   检测摩擦轮是否超速或低速，高温
// * @note   每次进入一个分支，完成分支任务后都必须return，避免一次性进入多个分支导致速度来回振荡
// */
//void Shoot_Speed_Self_Adapt(Shoot_t* shoot)
//{
//	static float speed;
//	
//	static uint16_t  more_cnt;                       //统计弹速高速次数，不包括超速
//	static uint16_t  less_cnt;                       //统计弹速低速次数

//	speed = shoot->judge_pkt.now_speed;
//	
//	//未打弹时直接退出，不做自适应
//	if(shoot->judge_pkt.shoot_freq == 0)              
//	{
//		more_cnt = 0;
//		less_cnt = 0;
//		return;
//	}
//	//第一发弹速可能不准确，不做自适应
//	else if(shoot->judge_pkt.shoot_freq > 0)
//	{
//		static uint16_t tick;
//    if (++tick < shoot->fric.config.adapt_config.first_bullet_shield_time) 
//		{
//			more_cnt = 0;
//			less_cnt = 0;
//			return;
//		}
//    tick = 0;
//	}
//	
//	//弹速在理想范围内不做自适应
//	if(speed >= (shoot->fric.config.adapt_config.ideal_speed_min - shoot->fric.config.adapt_config.ideal_death_value)   //0.05f是防止死区而导致速度来回跳动
//	 && speed <= (shoot->fric.config.adapt_config.ideal_speed_max + shoot->fric.config.adapt_config.ideal_death_value))
//	{
//		more_cnt = 0;
//		less_cnt = 0;
//		return;
//	}
//	
//	//弹速超速直接减速并退出，防止反向加速而导致速度来回震荡
//	if(speed >= shoot->fric.config.adapt_config.speed_max)
//	{
//		shoot->fric.config.base_config.normal_speed_target -= shoot->fric.config.adapt_config.overspeed_adjust_speed;
//		more_cnt = 0;
//		less_cnt = 0;
//		
//		return;
//	}
//	
//	//弹速低于理想弹速区间做自适应
//	if(speed < shoot->fric.config.adapt_config.ideal_speed_min)
//	{
//		less_cnt ++;               //开始统计低速次数
//		more_cnt = 0;
//		
//		//低速次数达标才允许加速
//		if(less_cnt >= shoot->fric.config.adapt_config.less_cnt_max)
//		{
//			less_cnt = 0;
//			//防止配置不当导致补偿角度不足 1 rpm
//			float low_add_speed = shoot->fric.config.adapt_config.low_adjust_value * shoot->fric.config.adapt_config.low_adjust_speed;
//			if(low_add_speed <= 1)
//			{
//				low_add_speed = 1;
//			}
//			shoot->fric.config.base_config.normal_speed_target += low_add_speed;
//			return;
//		}
//	}
//	
//	//弹速位于危险区间，达到一定次数才做自适应
//	if(speed > shoot->fric.config.adapt_config.ideal_speed_max && speed < shoot->fric.config.adapt_config.speed_max)
//	{
//		more_cnt ++;                         //开始统计高速次数
//		less_cnt = 0;
//		
//		if(more_cnt >= shoot->fric.config.adapt_config.more_cnt_max)
//		{
//			more_cnt = 0;
//			//避免配置不当而导致被除数为 0
//		  float divide = shoot->fric.config.adapt_config.speed_max - shoot->fric.config.adapt_config.ideal_speed_max;
//		  if(divide == 0)
//		  {
//		  	divide = 1;
//	  	}
//		
//		  //避免配置不当而导致补偿速度不足 1 rpm
//		  float high_minu_speed = shoot->fric.config.adapt_config.high_adjust_value * shoot->fric.config.adapt_config.high_adjust_speed;
//		  if(high_minu_speed <= 1)
//		  {
//		  	high_minu_speed = 1;
//		  }
//		    shoot->fric.config.base_config.normal_speed_target -= (speed - shoot->fric.config.adapt_config.ideal_speed_max)
//		                                                      / divide *high_minu_speed;                   
//	  }
//		return;                             
//	}
//	
//}


///**
//* @brief   发射机构睡眠
// */
//void Shoot_Sleep(Shoot_t* shoot)
//{
//	//状态全部睡眠，输出全为 0
//	shoot->fric.cmd.work_state =STOP;
//	shoot->fric.cmd.current_target = 0;
//	shoot->dial.cmd.current_target = 0;
//	shoot->dial.cmd.work_state = SLEEP;

//}

/**
* @brief   发射机构基本工作
 */
void Shoot_Base_Work(Shoot_t* shoot)
{
//	Fric_Block_Check(shoot);           //检查摩擦轮是否堵转
//	Fric_State_Check(shoot);           //检查摩擦轮是否速度不稳，高温
	
	Shoot_Work_State_Update(shoot);    //更新发射机构工作状态
	Shoot_Mode_Update(shoot);          //更新发射机构模式
	Dial_Work_State_Update(shoot);  //拨盘复位状态
	
//	switch (shoot->shoot_work_state)
//	{
//		case LOCKED:
//			Shoot_Sleep(shoot);             //发射机构睡眠
//		  break;
//		
//		case INITING:
//			Dial_Work_State_Update(shoot);  //拨盘复位状态
//		  break;
//		
//		case UNLOCK:
//			Dial_Work_State_Update(shoot);  //拨盘状态实时更新
//	    Shoot_Speed_Self_Adapt(shoot);  //弹速自适应
//		  break;

//	}
}


