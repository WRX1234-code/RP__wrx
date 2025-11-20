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
	shoot->shoot_flag.fire_flag = 0;
	shoot->shoot_flag.firing_flag = 0;
	shoot->shoot_flag.fric_block_flag = 0;
	shoot->shoot_flag.dial_block_flag = 0;
	shoot->shoot_flag.reset_flag = 1;
	shoot->shoot_flag.reset_adjust_flag = 1;
	shoot->shoot_flag.init_flag = 1;
	shoot->shoot_flag.reload_flag = 1;
	shoot->shoot_flag.fric_normal_speed_flag = 1;
	shoot->shoot_flag.fric_high_temp_flag = 0;
	
	/*在此处配置电机结构体config，oneshot_angle必须配置

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
	}

	else if(shoot->shoot_flag.reset_flag == 1&&shoot->shoot_flag.init_flag == 0)
	{
		shoot->shoot_work_state = INITING;                                  //初始化状态更新
		shoot->dial.cmd.work_state = RESETING;                              //拨盘进入复位状态
		shoot->dial.cmd.mode = DIAL_SPEED;
	}
	
	else if(shoot->shoot_flag.reset_flag == 1&&shoot->shoot_flag.init_flag == 1)
	{
		shoot->shoot_work_state = UNLOCK;                                   //开保险状态更新
    shoot->fric.cmd.work_state = RUN;                                   //摩擦轮开转
		if(shoot->shoot_flag.fric_high_temp_flag == 0)
		{
			//摩擦轮正常温度或恢复温度才给目标速度值
			shoot->fric.cmd.speed_target = shoot->fric.config.base_config.normal_speed_target;      
		}
		else if(shoot->shoot_flag.fric_high_temp_flag == 1)
		{
			shoot->fric.cmd.speed_target = shoot->fric.config.base_config.high_temp_speed_target;     //摩擦轮高温就降速                     
		}	
  }
}

/**
* @brief   发射机构模式实时更新
* @note   fire_flag,firing_flag置 1均在外部文件执行，分别由上升沿，高电平触发
 */
void Shoot_Mode_Update(Shoot_t* shoot)
{
	static uint8_t last_mode;
	static uint8_t last_state;
	
	if(shoot->shoot_flag.fire_flag == 1)
	{
		shoot->shoot_mode = SIMGLE_SHOT;
		shoot->dial.cmd.mode = DIAL_ANGLE;
		shoot->shoot_flag.reload_flag = 0;
		
		//连发速度环切单发角度环需要补偿小角度来保持最佳单发发射状态
		if(last_state == BURST && last_mode == DIAL_SPEED)
		{
			shoot->dial.cmd.angle_sum_target -= shoot->dial.config.base_config.switch_adjust_angle;
		}
	}
	else if(shoot->shoot_flag.firing_flag == 1)
	{
		shoot->shoot_mode = BURST;
		shoot->shoot_flag.reload_flag = 0;
		if(shoot->dial.config.base_config.burst_mode == DIAL_ANGLE)              //角度环连发
		{
			shoot->dial.cmd.mode = DIAL_ANGLE;
			//连发速度环切连发角度环也需要补偿角度
			if(last_mode == DIAL_SPEED)
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
uint8_t Dial_Block_Check(Dial_Rx_Info_t* info,Dial_Block_Config_t* config)
{
	uint8_t flag = 0;
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
	
	return flag;
}

/**
 * @brief   检测摩擦轮是否堵转
 * @param   info   - 拨盘实时数据
 * @param   config - 堵转判断阈值配置
 * @retval  1 - 堵转；0 - 正常
 * @note    阻塞时间单位：1 ms 调用周期
 */
uint8_t Fric_Block_Check(Shoot_t* shoot)
{
	uint8_t flag = 0;
	for(uint8_t i = 0;i < FRIC_LIST;i ++)
	{
		if(shoot->fric.info[i].speed < shoot->fric.config.block_config.speed_max
		&& shoot->fric.info[i].current > shoot->fric.config.block_config.current_min)
	  {
			if(shoot->fric.config.block_config.block_time[i] >= shoot->fric.config.block_config.block_time_max)
			{
				flag = 1;
				shoot->shoot_flag.fric_block_flag = 1;
			}
			else
			{
		  	shoot->fric.config.block_config.block_time[i] ++;
			}
	  }
		else
		{
			shoot->fric.config.block_config.block_time[i] = 0;
		}
	}
	
	return flag;
}

/**
 * @brief   拨盘实时状态更新
 */
void Dial_Work_State_Update(Shoot_t* shoot)
{
	static uint8_t work_time;                                //记录工作时间，想看状态变化可以将其提升至全局变量
	static uint16_t now_tick;                                //保存当前tick值
	static uint16_t last_tick;                               //保存上一次tick值
	 
	switch (shoot->dial.cmd.work_state)
	{
		case SLEEP:                                            //睡眠模式更新
			shoot->dial.cmd.speed_target = 0;
		  work_time = 0;
		
	 //只要INITING外部更新，可多次切换成复位模式
		if(shoot->shoot_work_state == INITING)                
			{
				shoot->dial.cmd.work_state = RESETING;
			}
			//需要同时符合缺弹跟摩擦轮不堵转才能补弹
			if(shoot->shoot_flag.reload_flag == 0 && shoot->shoot_flag.fric_block_flag == 0)   
			{
				shoot->dial.cmd.work_state = RELOAD;
				if(shoot->shoot_work_state == SIMGLE_SHOT)                
				{
					shoot->dial.cmd.angle_sum_target +=shoot->dial.config.base_config.oneshot_angle;
				}
				else if(shoot->shoot_work_state == BURST && shoot->dial.config.base_config.burst_mode == DIAL_ANGLE)
				{
					shoot->dial.cmd.angle_sum_target +=shoot->dial.config.base_config.oneshot_angle;
					last_tick = HAL_GetTick();
				}
				else if(shoot->shoot_work_state == BURST && shoot->dial.config.base_config.burst_mode == DIAL_SPEED)
				{
					shoot->dial.cmd.speed_target = shoot->dial.config.base_config.reload_speed;
				}
			}
		
		  break;
		
		case RESETING:                                            //复位状态更新
			
			//宏定义用于变换速度方向，方向取决于拨盘有无机械限位		 
		  shoot->dial.cmd.speed_target = -DIAL_MEC_LIMIT * shoot->dial.config.base_config.reset_speed;  
		
		  if(Dial_Block_Check(&shoot->dial.info,&shoot->dial.config.reset_block_config) == 1)   //拨盘堵转返回 1
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
			}
			//必须要初始化完成，复位完成，且拨盘角度环停下来才可以切换进睡眠模式
			if(shoot->shoot_flag.init_flag == 0 && shoot->shoot_flag.reset_adjust_flag == 0 && shoot->dial.info.speed == 0)
			{
				shoot->dial.cmd.work_state = SLEEP;
				shoot->dial.cmd.speed_target = 0;
				shoot->shoot_flag.init_flag = 1;
				shoot->shoot_flag.reset_adjust_flag = 1;
				//记录拨盘起始角度和，用于后续计算补偿角
				shoot->dial.angle_sum_start = shoot->dial.info.angle_sum;         
			}
		
		  break;
		
		case RELOAD:                             //补弹模式更新
		  shoot->dial.cmd.reload_sche = (float)(shoot->dial.cmd.angle_sum_target - shoot->dial.info.angle_sum) 
		                                 / (float)shoot->dial.config.base_config.oneshot_angle;	
		
			if(shoot->shoot_work_state == SIMGLE_SHOT && shoot->shoot_flag.fire_flag == 1)    //单发模式且开火标志位已触发
			{
				work_time ++;
				if(Dial_Block_Check(&shoot->dial.info,&shoot->dial.config.reload_angle_block_config) == 1)
			  {
			  	shoot->dial.cmd.work_state = RECOIL;
				  shoot->dial.cmd.angle_sum_target -=shoot->dial.config.base_config.oneshot_angle;
				  shoot->shoot_flag.dial_block_flag = 1;
					//清零工作时间，防止后面误入分支
				  work_time = 0;                                   
					shoot->shoot_flag.fire_flag = 0;
					shoot->shoot_flag.reload_flag = 1;
			  }
				//超时退出
			  else if(work_time >= shoot->dial.config.base_config.state_work_time_max)
			  {
				  shoot->dial.cmd.work_state = SLEEP;
				  shoot->dial.cmd.speed_target = 0;
					work_time = 0;
					shoot->shoot_flag.fire_flag = 0;
					shoot->shoot_flag.reload_flag = 1;
			  }
				//拨盘完成角度环，停下来才切换睡眠模式
				else if(shoot->dial.info.speed == 0)
				{
					shoot->dial.cmd.work_state = SLEEP;
					shoot->dial.cmd.speed_target = 0;
					work_time = 0;
					shoot->shoot_flag.fire_flag = 0;
					shoot->shoot_flag.reload_flag = 1;
				}
		  }
			else if(shoot->shoot_work_state == BURST)                     //连发模式
			{
				if(shoot->shoot_flag.firing_flag == 1)
				{
					if(shoot->dial.config.base_config.burst_mode == DIAL_SPEED)
				  {
						//计算补偿角，用于后续速度环切角度环时拨盘回转以补偿超出角度环目标值集合的角度
					  shoot->dial.config.base_config.switch_adjust_angle = (shoot->dial.info.angle_sum - shoot->dial.angle_sum_start)
				                                                        % shoot->dial.config.base_config.oneshot_angle;
					  shoot->dial.cmd.angle_sum_target = shoot->dial.info.angle_sum;
			  	  if(Dial_Block_Check(&shoot->dial.info,&shoot->dial.config.reload_speed_block_config) == 1)
			      {
			        shoot->dial.cmd.work_state = RECOIL;
				      shoot->dial.cmd.mode = DIAL_ANGLE;
				      shoot->dial.cmd.angle_sum_target -=shoot->dial.config.base_config.switch_adjust_angle;  //堵转也要调整角度
				      shoot->shoot_flag.dial_block_flag = 1;
						  shoot->shoot_flag.firing_flag = 0;
							shoot->shoot_flag.reload_flag = 1;
				      work_time = 0;
			      }
				  }
				  else if(shoot->dial.config.base_config.burst_mode == DIAL_ANGLE)
				  {
						
						now_tick = HAL_GetTick();
						//判断连发周期是否到达在决定是否打弹
						if(now_tick - last_tick >= shoot->dial.config.base_config.burst_period)
						{
							shoot->dial.cmd.angle_sum_target += shoot->dial.config.base_config.oneshot_angle;
							shoot->shoot_flag.reload_flag = 0;
							work_time ++;
							last_tick = now_tick;
						}
						//拨盘在周期内完成角度环并停下
						else if(now_tick - last_tick < shoot->dial.config.base_config.burst_period 
							      && shoot->dial.info.speed == 0)
						{
							work_time = 0;
							shoot->shoot_flag.reload_flag = 1;
						}
						
				    if(Dial_Block_Check(&shoot->dial.info,&shoot->dial.config.reload_angle_block_config) == 1)
			      {
			  	    shoot->dial.cmd.work_state = RECOIL;
							shoot->dial.cmd.mode = DIAL_ANGLE;
				      shoot->dial.cmd.angle_sum_target -=shoot->dial.config.base_config.oneshot_angle;
				      shoot->shoot_flag.dial_block_flag = 1;
				      work_time = 0;
					    shoot->shoot_flag.firing_flag = 0;
							shoot->shoot_flag.reload_flag = 1;
			      }
						//超时退出
			      else if(work_time >= shoot->dial.config.base_config.state_work_time_max)
			      {
				      shoot->dial.cmd.work_state = SLEEP;
				      shoot->dial.cmd.speed_target = 0;
					    work_time = 0;
					    shoot->shoot_flag.firing_flag = 0;
							shoot->shoot_flag.reload_flag = 1;
			      }
			    }
					else if(shoot->shoot_flag.firing_flag == 0)
					{
						shoot->dial.cmd.work_state = SLEEP;
						shoot->dial.cmd.speed_target = 0;
						shoot->shoot_flag.reload_flag = 1;
		        work_time = 0;
					}
						
				}	 
			} 
		
		  break;
		
		case RECOIL:                                                           //堵转状态更新
			shoot->dial.config.reload_angle_block_config.block_time = 0;         //清零堵转记时
		  shoot->dial.config.reload_speed_block_config.block_time = 0;
		  //成功退弹
			if(shoot->dial.info.angle_sum <= shoot->dial.cmd.angle_sum_target)
			{
				shoot->dial.cmd.work_state = SLEEP;
				shoot->dial.cmd.speed_target = 0;
				work_time = 0; 
			}
			//超时退出
		  else if(work_time >= shoot->dial.config.base_config.state_work_time_max)
			{
				shoot->dial.cmd.work_state = SLEEP;
				shoot->dial.cmd.speed_target = 0;
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
 * @brief   检测摩擦轮是否超速或低速，高温
 */
void Fric_State_Check(Shoot_t* shoot)
{
	//这个 k 在宏定义中有指出
	int8_t k = 1;
	//宏定义顶替一段可能较长的，用于通过改变 k 为 1和 -1 的逻辑，从而改变速度方向
	FRIC_SPEED_DATA_DIRECTION_MENAGE
	
	uint8_t i;
	for(i = 0;i < FRIC_LIST;i ++)                //遍历求差，便于debug
	{
		shoot->fric.check.speed_err[i] = abs(shoot->fric.info[i].speed - k*shoot->fric.config.base_config.normal_speed_target);
		shoot->fric.check.temp_err[i] = shoot->fric.info[i].temperature - shoot->fric.config.base_config.temp_max;
	}
	//枚举选出对应的摩擦轮分别查看情况
	#if FRIC_NUM == 6
	
	if(shoot->fric.check.speed_err[FRIC_B_UP] < shoot->fric.config.base_config.speed_err_max
		&& shoot->fric.check.speed_err[FRIC_B_R] < shoot->fric.config.base_config.speed_err_max
    && shoot->fric.check.speed_err[FRIC_B_L] < shoot->fric.config.base_config.speed_err_max
    && shoot->fric.check.speed_err[FRIC_F_UP] < shoot->fric.config.base_config.speed_err_max
    && shoot->fric.check.speed_err[FRIC_F_R] < shoot->fric.config.base_config.speed_err_max
    && shoot->fric.check.speed_err[FRIC_F_L] < shoot->fric.config.base_config.speed_err_max	)

  #elif FRIC_NUM == 3	
	if(shoot->fric.check.speed_err[FRIC_UP] < shoot->fric.config.base_config.speed_err_max
		&& shoot->fric.check.speed_err[FRIC_R] < shoot->fric.config.base_config.speed_err_max
	  && shoot->fric.check.speed_err[FRIC_L] < shoot->fric.config.base_config.speed_err_max)

  #elif FRIC_NUM == 2	
	if(shoot->fric.check.speed_err[FRIC_R] < shoot->fric.config.base_config.speed_err_max
		&& shoot->fric.check.speed_err[FRIC_L] < shoot->fric.config.base_config.speed_err_max)
	#endif
	{
		shoot->shoot_flag.fric_normal_speed_flag = 1;
	}
	else 
	{
		shoot->shoot_flag.fric_normal_speed_flag = 0;
	}	
	
	
	#if FRIC_NUM == 6
	
	if(shoot->fric.check.temp_err[FRIC_B_UP] < shoot->fric.config.base_config.temp_err_max
		&& shoot->fric.check.temp_err[FRIC_B_R] < shoot->fric.config.base_config.temp_err_max
    && shoot->fric.check.temp_err[FRIC_B_L] < shoot->fric.config.base_config.temp_err_max
    && shoot->fric.check.temp_err[FRIC_F_UP] < shoot->fric.config.base_config.temp_err_max
    && shoot->fric.check.temp_err[FRIC_F_R] < shoot->fric.config.base_config.temp_err_max
    && shoot->fric.check.temp_err[FRIC_F_L] < shoot->fric.config.base_config.temp_err_max	)

  #elif FRIC_NUM == 3	
	if(shoot->fric.check.temp_err[FRIC_UP] < shoot->fric.config.base_config.temp_err_max
		&& shoot->fric.check.temp_err[FRIC_R] < shoot->fric.config.base_config.temp_err_max
	  && shoot->fric.check.temp_err[FRIC_L] < shoot->fric.config.base_config.temp_err_max)

  #elif FRIC_NUM == 2	
	if(shoot->fric.check.temp_err[FRIC_R] < shoot->fric.config.base_config.temp_err_max
		&& shoot->fric.check.temp_err[FRIC_L] < shoot->fric.config.base_config.temp_err_max)
	#endif
	{
		shoot->shoot_flag.fric_high_temp_flag = 0;
	}
	else 
	{
		shoot->shoot_flag.fric_high_temp_flag = 1;
	}	
	
}

/**
* @brief   发射机构睡眠
 */
void Shoot_Sleep(Shoot_t* shoot)
{
	//状态全部睡眠，输出全为 0
	shoot->fric.cmd.work_state =STOP;
	shoot->fric.cmd.current_target = 0;
	shoot->dial.cmd.current = 0;
	shoot->dial.cmd.work_state = SLEEP;

}

/**
* @brief   发射机构基本工作
 */
void Shoot_Base_Work(Shoot_t* shoot)
{
	Fric_Block_Check(shoot);           //检查摩擦轮是否堵转
	Fric_State_Check(shoot);           //检查摩擦轮是否速度不稳，高温
	
	Shoot_Work_State_Update(shoot);    //更新发射机构工作状态
	Shoot_Mode_Update(shoot);          //更新发射机构模式
	 
	switch (shoot->shoot_work_state)
	{
		case LOCKED:
			Shoot_Sleep(shoot);
		  break;
		
		case INITING:
		case UNLOCK:
			Dial_Work_State_Update(shoot);
		  break;

	}
}


