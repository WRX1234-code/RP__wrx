#include "shoot_base.h"
#include <stdlib.h>

Shoot_t shoot;

/**
 * @brief   初始化发射机构状态与默认参数
 * @note    仅赋值状态，不会启动电机；配置项在外部 json 加载
 * @warning 必须在所有电机上电前调用
 */
void Shoot_Init(Shoot_t* shoot)
{
	//初始化工作状态
	shoot->shoot_work_state = LOCKED;
	//初始化发射机构模式
	shoot->shoot_mode = CEASEFIRE;
	//初始化命令
	shoot->dial.cmd.work_state = SLEEP;
	shoot->fric.cmd.work_state = STOP;
	//初始化标志位
	shoot->shoot_flag.reset_flag = 1;
	shoot->shoot_flag.is_not_fire_flag = 0;
	shoot->shoot_flag.dial_block_flag = 0;
	shoot->shoot_flag.reset_adjust_flag = 1;
	shoot->shoot_flag.init_flag = 1;
	shoot->shoot_flag.reload_flag = 1;
	

	/*在此处配置电机结构体config，不得有漏配置

	*/
}

/**
 * @brief   更新拨盘工作状态
 * @note    内部使用 static 变量保存节拍计数，不可重入
 */
void Shoot_Work_State_Update(Shoot_t* shoot)
{
	if(shoot->shoot_flag.reset_flag == 0&&shoot->shoot_flag.init_flag == 0)
	{
		shoot->shoot_work_state = LOCKED;                                   //关保险状态更新
		shoot->shoot_mode = CEASEFIRE;
		shoot->fric.cmd.work_state = STOP;
		shoot->dial.cmd.work_state = SLEEP;
	}

	else if(shoot->shoot_flag.reset_flag == 1&&shoot->shoot_flag.init_flag == 0)
	{
		shoot->shoot_work_state = INITING;                                  //初始化状态更新
		shoot->dial.cmd.work_state = RESETING;                              //拨盘进入复位状态
		shoot->dial.cmd.mode = DIAL_SPEED;
		shoot->fric.cmd.work_state = RUN;
	}
	
	else if(shoot->shoot_flag.reset_flag == 1&&shoot->shoot_flag.init_flag == 1)
	{
		shoot->shoot_work_state = UNLOCK;                                   //开保险状态更新
    shoot->fric.cmd.work_state = RUN;                                   //摩擦轮开转
		

  }
}

/**
* @brief   发射机构模式实时更新
* @note    fire_mode_flag外部文件更新数值     
 */
void Shoot_Mode_Update(Shoot_t* shoot)
{
	static uint8_t last_mode;
	static uint8_t last_state;
	
	if(shoot->shoot_flag.fire_mode_flag == 0)
	{
		shoot->shoot_mode = SIMGLE_SHOT;
		shoot->dial.cmd.mode = DIAL_ANGLE;
		shoot->shoot_flag.reload_flag = 0;
		
		//连发命令取消时拨盘原地停下的前提，连发速度环切单发角度环需要补偿小角度来保持最佳单发发射状态
		if(last_state == BURST && last_mode == DIAL_SPEED && shoot->dial.speed_stop_mode == STAND)
		{
			shoot->dial.cmd.angle_sum_target -= shoot->dial.config.base_config.switch_adjust_angle;
		}
	}
	else if(shoot->shoot_flag.fire_mode_flag == 1)
	{
		shoot->shoot_mode = BURST;
		shoot->shoot_flag.reload_flag = 0;
		if(shoot->dial.config.base_config.burst_mode == DIAL_ANGLE)              //角度环连发
		{
			shoot->dial.cmd.mode = DIAL_ANGLE;
			//连发命令取消时拨盘原地停下的前提，连发速度环切连发角度环也需要补偿角度
			if(last_mode == DIAL_SPEED && shoot->dial.speed_stop_mode == STAND)
			{
				shoot->dial.cmd.angle_sum_target -= shoot->dial.config.base_config.switch_adjust_angle;
			}
		}
		else if(shoot->dial.config.base_config.burst_mode == DIAL_SPEED)          //速度环连发
		{
			shoot->dial.cmd.mode = DIAL_SPEED;
			shoot->dial.cmd.angle_sum_target = shoot->dial.info.angle_sum;          //实时更新角度和目标值
		}
	}
	else
	{
		shoot->shoot_mode = CEASEFIRE;                                           //停火模式
	}
	
	last_state = shoot->shoot_mode;
	last_mode = shoot->dial.cmd.mode;
}

/**
 * @brief   检测拨盘是否堵转
 * @param   info   - 拨盘实时数据
 * @param   config - 堵转判断阈值配置
 * @retval  1 - 堵转；0 - 正常
 * @note    阻塞时间单位：1 ms 调用周期
 */
uint8_t Dial_Block_Check(Dial_Rx_Info_t* info,Dial_Block_Config_t* config,Dial_Tx_Cmd_t* cmd)
{
	uint8_t flag = 0;
	if(config->block_judge_type == 0)   //堵转判断方法1
	{
		if(info->speed < config->speed_max && info->current > config->current_min)
    {
		  if(config->block_time >= config->block_time_max)
		  {
		  	flag = 1;
		  }
		  else
		  {
		    config->block_time ++;
	    } 
	  }
	  else
	  {
		   config->block_time = 0;
	  }
	}
	else if(config->block_judge_type == 1 && cmd->mode == DIAL_ANGLE)    //堵转判断方法2，只适用于角度环
	{
		config->angle_sum_err_integral += config->integral_value * (cmd->angle_sum_target - info->angle_sum);
		if(config->angle_sum_err_integral >= config->angle_sum_err_integral_max)
		{
			flag = 1;
			config->angle_sum_err_integral = config->angle_sum_err_integral_max;
		}
	}
	
	
	return flag;
}



/**
 * @brief   拨盘实时状态更新
 * @note    is_not_fire_flag外部文件更新
 */
void Dial_Work_State_Update(Shoot_t* shoot)
{
	static uint8_t work_time;                                //记录工作时间，想看状态变化可以将其提升至全局变量
	static uint16_t now_tick;                                //保存当前tick值
	static uint16_t last_tick;                               //保存上一次tick值
	 
	switch (shoot->dial.cmd.work_state)
	{
		case SLEEP:                                              //睡眠模式更新，只卸力
			shoot->dial.cmd.current = 0;
		  break;
		
		case WAITING:                                            //等待模式更新，角度环
		  shoot->dial.cmd.mode = DIAL_ANGLE;
		  work_time = 0;
		
	    //只要INITING外部更新，可多次切换成复位模式
		  if(shoot->shoot_work_state == INITING)                
			{
				shoot->dial.cmd.work_state = RESETING;
			}
			//需要同时符合缺弹跟允许开火才能补弹
			if(shoot->shoot_flag.reload_flag == 0 && shoot->shoot_flag.is_not_fire_flag == 1)   
			{
				shoot->dial.cmd.work_state = RELOAD;
				if(shoot->shoot_work_state == SIMGLE_SHOT)                
				{
					shoot->dial.cmd.angle_sum_target +=shoot->dial.config.base_config.oneshot_angle;
				}
				else if(shoot->shoot_work_state == BURST && shoot->dial.config.base_config.burst_mode == DIAL_ANGLE)
				{
					shoot->dial.cmd.angle_sum_target +=shoot->dial.config.base_config.oneshot_angle;
					shoot->dial.cmd.mode = DIAL_ANGLE;
					
					last_tick = HAL_GetTick();
				}
				else if(shoot->shoot_work_state == BURST && shoot->dial.config.base_config.burst_mode == DIAL_SPEED)
				{
					shoot->dial.cmd.speed_target = shoot->dial.config.base_config.reload_speed;
					shoot->dial.cmd.mode = DIAL_SPEED;
				}
			}
		
		  break;
		
		case RESETING:                                            //复位状态更新
			work_time ++;
		
			//宏定义用于变换速度方向，方向取决于拨盘有无机械限位		 
		  shoot->dial.cmd.speed_target = -DIAL_MEC_LIMIT * shoot->dial.config.base_config.reset_speed;  
		
		  if(Dial_Block_Check(&shoot->dial.info,&shoot->dial.config.reset_block_config,&shoot->dial.cmd) == 1)   //拨盘堵转返回 1
			{
				//对拨盘回转调整角度限幅，防止过大
				if(shoot->dial.config.base_config.reset_adjust_angle > shoot->dial.config.base_config.reset_adjust_angle_max)
				{
					shoot->dial.config.base_config.reset_adjust_angle = shoot->dial.config.base_config.reset_adjust_angle_max;
				}
				else if(shoot->dial.config.base_config.reset_adjust_angle < -shoot->dial.config.base_config.reset_adjust_angle_max)
				{
					shoot->dial.config.base_config.reset_adjust_angle = -shoot->dial.config.base_config.reset_adjust_angle_max;
				}
				
		    shoot->dial.cmd.angle_sum_target += DIAL_MEC_LIMIT * shoot->dial.config.base_config.reset_adjust_angle;
		
				shoot->dial.cmd.mode = DIAL_ANGLE;
				//复位调整标志位置 0，下次复位才会提前置 1
				shoot->shoot_flag.reset_adjust_flag = 0;     
        
        work_time = 0;		
				//堵转参考数值清零，防止出问题
        shoot->dial.config.reset_block_config.block_time = 0;                        
        shoot->dial.config.reset_block_config.angle_sum_err_integral = 0;				
			}
			//超时退出，进入等待模式
			else if(work_time >= shoot->dial.config.base_config.reset_work_time_max)
			{
				shoot->dial.cmd.work_state = WAITING;
		    shoot->dial.cmd.mode = DIAL_ANGLE;
				shoot->shoot_flag.init_flag = 1;
				shoot->shoot_flag.reset_adjust_flag = 1;
				//记录拨盘起始角度和，用于后续计算补偿角
				shoot->dial.angle_sum_start = shoot->dial.info.angle_sum;     
        
        shoot->dial.config.reset_block_config.block_time = 0;
        shoot->dial.config.reset_block_config.angle_sum_err_integral = 0;							
			}
			
			//初始化完成，复位完成，且拨盘角度环停下来切换进等待模式
			if(shoot->shoot_flag.init_flag == 0 && shoot->shoot_flag.reset_adjust_flag == 0 && shoot->dial.info.speed == 0)
			{
				shoot->dial.cmd.work_state = WAITING;
				shoot->dial.cmd.mode = DIAL_ANGLE;
				shoot->shoot_flag.init_flag = 1;
				shoot->shoot_flag.reset_adjust_flag = 1;
				//记录拨盘起始角度和，用于后续计算补偿角
				shoot->dial.angle_sum_start = shoot->dial.info.angle_sum;  				
   
        shoot->dial.config.reset_block_config.block_time = 0;
        shoot->dial.config.reset_block_config.angle_sum_err_integral = 0;			
			}
		
		  break;
		
		case RELOAD:                             //补弹模式更新
			
		  switch (shoot->dial.cmd.mode)
		  {
		  	case DIAL_ANGLE:
			  	work_time ++;
          //角度环连发有发弹周期
          if(shoot->shoot_mode == BURST)
				  {
					  now_tick = HAL_GetTick();
			  	  //判断连发周期是否到达在决定是否打弹
				    if(now_tick - last_tick >= shoot->dial.config.base_config.burst_period)
				    {
				  	  shoot->dial.cmd.angle_sum_target += shoot->dial.config.base_config.oneshot_angle;
				  	  shoot->shoot_flag.reload_flag = 0;
					    work_time ++;
					    last_tick = now_tick;                              //实时更新tick
				    }
				    //拨盘在周期内完成角度环并停下
				    else if(now_tick - last_tick < shoot->dial.config.base_config.burst_period 
				 	          && shoot->dial.info.speed == 0)
				    {
					    work_time = 0;
				    	shoot->shoot_flag.reload_flag = 1;
							
							shoot->dial.config.reload_angle_block_config.block_time = 0;
              shoot->dial.config.reload_angle_block_config.angle_sum_err_integral = 0;			
            }
			  	}					
				
				   //实时更新拨弹进度
				  shoot->dial.cmd.reload_sche = (float)(shoot->dial.cmd.angle_sum_target - shoot->dial.info.angle_sum) 
		                                   / (float)shoot->dial.config.base_config.oneshot_angle;	
				
				  //堵转处理切退弹模式
				  if(Dial_Block_Check(&shoot->dial.info,&shoot->dial.config.reload_angle_block_config,&shoot->dial.cmd) == 1)
			    {
			  	  shoot->dial.cmd.work_state = RECOIL;
				    shoot->dial.cmd.angle_sum_target -=shoot->dial.config.base_config.oneshot_angle;
				    shoot->shoot_flag.dial_block_flag = 1;
				  	//清零工作时间，防止后面误入分支
				    work_time = 0;                                   
					  shoot->shoot_flag.is_not_fire_flag = 0;
					  shoot->shoot_flag.reload_flag = 1;
			    }
				  //超时退出
			    else if(work_time >= shoot->dial.config.base_config.state_work_time_max)
			    {
				    shoot->dial.cmd.work_state = WAITING;
					  shoot->dial.cmd.mode = DIAL_ANGLE;
					  work_time = 0;
				  	shoot->shoot_flag.is_not_fire_flag = 0;
				  	shoot->shoot_flag.reload_flag = 1;
						
						shoot->dial.config.reload_angle_block_config.block_time = 0;
            shoot->dial.config.reload_angle_block_config.angle_sum_err_integral = 0;			
			    }
				  //拨盘完成角度环，停下来才切换睡眠模式
				  else if(shoot->dial.info.speed == 0)
				  {
					  shoot->dial.cmd.work_state = WAITING;
					  shoot->dial.cmd.mode = DIAL_ANGLE;
					  work_time = 0;
					  shoot->shoot_flag.is_not_fire_flag = 0;
					  shoot->shoot_flag.reload_flag = 1;
						
						shoot->dial.config.reload_angle_block_config.block_time = 0;
            shoot->dial.config.reload_angle_block_config.angle_sum_err_integral = 0;			
				  }
		    
	        break;
				
		    case DIAL_SPEED:
				
			    if(shoot->shoot_flag.is_not_fire_flag == 1)
				  {
				  	//计算补偿角，用于后续速度环切角度环时拨盘回转以补偿超出角度环目标值集合的角度
				    shoot->dial.config.base_config.switch_adjust_angle = (shoot->dial.info.angle_sum - shoot->dial.angle_sum_start)
				                                                         % shoot->dial.config.base_config.oneshot_angle;
						
				    //速度环与角度环计算方式稍微不同
				    shoot->dial.cmd.reload_sche = (float)(shoot->dial.config.base_config.oneshot_angle - shoot->dial.config.base_config.switch_adjust_angle) 
		                                      / (float)shoot->dial.config.base_config.oneshot_angle;	
						
				    //实时更新角度和目标值
				    shoot->dial.cmd.angle_sum_target = shoot->dial.info.angle_sum;
				    //堵转处理
			      if(Dial_Block_Check(&shoot->dial.info,&shoot->dial.config.reload_speed_block_config,&shoot->dial.cmd) == 1)
			      {
			        shoot->dial.cmd.work_state = RECOIL;
				      shoot->dial.cmd.mode = DIAL_ANGLE;
				      shoot->dial.cmd.angle_sum_target -=shoot->dial.config.base_config.oneshot_angle;  //堵转也要调整一弹丸角度，后面会补回来
				      shoot->shoot_flag.dial_block_flag = 1;
				      shoot->shoot_flag.is_not_fire_flag = 0;
				      shoot->shoot_flag.reload_flag = 1;
				      work_time = 0;
			      }
				  }
				  else if(shoot->shoot_flag.is_not_fire_flag == 0)       //连发开火停止
				  {
					  shoot->dial.cmd.work_state = WAITING;
					  shoot->dial.cmd.mode = DIAL_ANGLE;
					  shoot->shoot_flag.reload_flag = 1;
		        work_time = 0;
						//速度环连发停止时有四种归位模式
						switch (shoot->dial.speed_stop_mode)
						{
						  case STAND:                //原地不动，此时目标值已经达到
							 break;
							
							case FORWARD:              //往前转动，达到前面的角度环集合目标值
								shoot->dial.cmd.angle_sum_target +=(shoot->dial.config.base_config.oneshot_angle - shoot->dial.config.base_config.switch_adjust_angle);
							  break;
							
							case BACKWARD:             //往后转动，达到后面的角度环集合目标值
								shoot->dial.cmd.angle_sum_target -= shoot->dial.config.base_config.switch_adjust_angle;
							  break;
							
							case ROUNDING_UP:          //四舍五入，距离哪边近就去哪边
								if(2 * shoot->dial.config.base_config.switch_adjust_angle > shoot->dial.config.base_config.oneshot_angle)
								{
									shoot->dial.cmd.angle_sum_target +=(shoot->dial.config.base_config.oneshot_angle - shoot->dial.config.base_config.switch_adjust_angle);
								}
								else if(2 * shoot->dial.config.base_config.switch_adjust_angle <= shoot->dial.config.base_config.oneshot_angle)
								{
									shoot->dial.cmd.angle_sum_target -= shoot->dial.config.base_config.switch_adjust_angle;
								}
								break;
						}
				  }
			    break;
			 }
				  
		   break;	
				 
		case RECOIL:                                                           //堵转状态更新
			//清零堵转参考值
			shoot->dial.config.reload_angle_block_config.block_time = 0;         
      shoot->dial.config.reload_angle_block_config.angle_sum_err_integral = 0;			
		  shoot->dial.config.reload_speed_block_config.block_time = 0;
		  //成功退弹
			if(shoot->dial.info.angle_sum <= shoot->dial.cmd.angle_sum_target)
			{
				shoot->dial.cmd.work_state = WAITING;
				shoot->dial.cmd.mode = DIAL_ANGLE;
				work_time = 0; 
				
			}
			//超时退出
		  else if(work_time >= shoot->dial.config.base_config.state_work_time_max)
			{
				shoot->dial.cmd.work_state = WAITING;
				shoot->dial.cmd.mode = DIAL_ANGLE;
				work_time = 0; 
			}
			else
			{
				work_time ++;
			}

		  break;
	}
	
}


/**
* @brief   发射机构基本工作
 */
void Shoot_Base_Work(Shoot_t* shoot)
{
	
	Shoot_Work_State_Update(shoot);    //更新发射机构工作状态
	Shoot_Mode_Update(shoot);          //更新发射机构模式
	Dial_Work_State_Update(shoot);  //拨盘复位状态
	
}}


