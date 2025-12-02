#include "shoot_base.h"
#include <stdlib.h>

Shoot_t shoot;

/*--------------------------------对内API定义----------------------------------*/

/**
 * @brief   半圈处理
 * @note    一般用于绝对角度，处理过边界问题
 */
static DIAL_ANGLE_ERR_DATA_TYPE Half_Cir_Handle(DIAL_ANGLE_ERR_DATA_TYPE err)
{
	if(err > (DIAL_ANGLE_MAX - DIAL_ANGLE_MIN) / 2)
	{
		err += DIAL_ANGLE_MIN - DIAL_ANGLE_MAX;
	}
	else if(err <= (DIAL_ANGLE_MIN - DIAL_ANGLE_MAX) / 2)
	{
		err += DIAL_ANGLE_MAX - DIAL_ANGLE_MIN;
	}
	return err;
}


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
	
	//未初始化
	if(last_angle == 0 && now_angle == 0)
	{
		angle_err = 0;
	}
	else
  {
		angle_err = now_angle - last_angle;
	}
	
	shoot->misc.angle_sum += Half_Cir_Handle(angle_err); 
	
  	
	last_angle = now_angle;
}                                   
	


/**
 * @brief   绝对角度归一化
 * @note    只用于绝对角度，避免超出绝对角度的取值范围
 */
static DIAL_ANGLE_DATA_TYPE Absolute_Angle_Wrap(DIAL_ANGLE_DATA_TYPE unwraped_angle)
{
	DIAL_ANGLE_ERR_DATA_TYPE angle = unwraped_angle; 
	if(DIAL_IS_ANSOLUTE_ANGLE)
	{
		
		if(angle > DIAL_ANGLE_MAX - 1)
		{
			angle -= DIAL_ANGLE_MAX - DIAL_ANGLE_MIN;
		}
		else if(angle < -DIAL_ANGLE_MAX + 1)
		{
			angle += DIAL_ANGLE_MAX - DIAL_ANGLE_MIN;
		}
		
	}
	else
	{
		angle = unwraped_angle;
	}
	return angle;
}

/**
 * @brief   上弹和退弹状态下角度环的目标角度转化
 */
static void Angle_Target_Switch(Shoot_t* shoot)
{
	#if DIAL_IS_ANSOLUTE_ANGLE
	if(shoot->cmd.dial_tx_cmd.work_state == RELOAD)
	{
	  shoot->cmd.dial_tx_cmd.angle_target = shoot->misc.front_absolute_angle_target;
	}
	else if(shoot->cmd.dial_tx_cmd.work_state == RECOIL)
	{
		shoot->cmd.dial_tx_cmd.angle_target = shoot->misc.behind_absolute_angle_target;
	}
	
	#else
	if(shoot->cmd.dial_tx_cmd.work_state == RELOAD)
	{
	  shoot->cmd.dial_tx_cmd.angle_target += shoot->info.cfg_rx_info.base_cfg_info.oneshot_angle;
	}
	else if(shoot->cmd.dial_tx_cmd.work_state == RECOIL)
	{
		shoot->cmd.dial_tx_cmd.angle_target -= shoot->info.cfg_rx_info.base_cfg_info.oneshot_angle;
	}
	#endif
}

/**
 * @brief   根据设定的绝对角度偏置角度初始化角度环目标值集合
* @note    偏置角度不要太大，拨片数量不要太离谱
 */
static void Absolute_Angle_Target_Init(Shoot_t* shoot)
{
	uint8_t i;
	uint32_t pusher_num;
	DIAL_ANGLE_ERR_DATA_TYPE err;
	
	
	//对拨片数量限幅
	if(DIAL_PUSHER_NUM <= 2)    //2片的话夹角很难快速供弹，负数片的话太抽象
	{
		pusher_num = 3;
	}
	else if(DIAL_PUSHER_NUM > DIAL_ANGLE_MAX)      //更加抽象 
	{
		pusher_num = DIAL_ANGLE_MAX;
	}
	else
  {
		pusher_num = DIAL_PUSHER_NUM;
	}
	//对偏置角度限幅
	if(shoot->info.cfg_rx_info.base_cfg_info.reset_offist_angle > shoot->info.cfg_rx_info.base_cfg_info.oneshot_angle)
	{
		shoot->info.cfg_rx_info.base_cfg_info.reset_offist_angle = shoot->info.cfg_rx_info.base_cfg_info.oneshot_angle;
	}
	else if(shoot->info.cfg_rx_info.base_cfg_info.reset_offist_angle < -shoot->info.cfg_rx_info.base_cfg_info.oneshot_angle)
	{
		shoot->info.cfg_rx_info.base_cfg_info.reset_offist_angle = -shoot->info.cfg_rx_info.base_cfg_info.oneshot_angle;
	}
	
	DIAL_ANGLE_DATA_TYPE abs_err[pusher_num];
	DIAL_ANGLE_DATA_TYPE angle_target[pusher_num];
	uint8_t err_min_cnt = 0;
	
	
	for(i = 0;i < pusher_num ;i ++)          //确定角度环目标值集合,计算角度误差准备排序
	{
		angle_target[i] = shoot->info.cfg_rx_info.base_cfg_info.reset_offist_angle + i * shoot->info.cfg_rx_info.base_cfg_info.oneshot_angle;
	  angle_target[i] = Absolute_Angle_Wrap(angle_target[i]);
		
		abs_err[i] = abs(angle_target[i] - shoot->info.rt_rx_info.dial_info.angle);
		if(abs_err[i] <= abs_err[err_min_cnt])
		{
			err_min_cnt = i;
		}
	}
	for(i = 0;i < pusher_num;i ++)           //角度目标值开始排序
	{
		uint8_t j = err_min_cnt + i;
		if(j > pusher_num)
		{
			j -= pusher_num;
		}
		shoot->misc.absolute_angle_target[i] = angle_target[j]; 
	}
}

/**
 * @brief   绝对角度当前角度的前后角度环目标值转换
 */
static void Absolute_Angle_Target_Transfor(Shoot_t* shoot)
{
	uint8_t i,j;
	uint32_t pusher_num;
	
	//对拨片数量限幅
	if(DIAL_PUSHER_NUM <= 2)    //2片的话夹角很难快速供弹，负数片的话太抽象
	{
		pusher_num = 3;
	}
	else if(DIAL_PUSHER_NUM > DIAL_ANGLE_MAX)      //更加抽象 
	{
		pusher_num = DIAL_ANGLE_MAX;
	}
	else
  {
		pusher_num = DIAL_PUSHER_NUM;
	}
	
	for(i = 0;i < pusher_num ;i ++)
	{
		if(i + 1 >= pusher_num)      //防止溢出
		{
			j = 0;
		}
		else                              //防止溢出
		{
			j = i + 1;
		}			
		
		if(shoot->misc.absolute_angle_target[i] <= shoot->info.rt_rx_info.dial_info.angle 
			&& shoot->misc.absolute_angle_target[j] > shoot->info.rt_rx_info.dial_info.angle)
		{
				shoot->misc.behind_absolute_angle_target = shoot->misc.absolute_angle_target[i];
				shoot->misc.front_absolute_angle_target = shoot->misc.absolute_angle_target[j];
		}
	}
}


/*--------------------------------对外API定义----------------------------------*/

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
	
	//  /*在此处配置电机结构体config，不得有漏配置
	
	//拨盘基本配置
	shoot->info.cfg_rx_info.base_cfg_info.reset_angle_work_time_max =; 
	
	shoot->info.cfg_rx_info.base_cfg_info.oneshot_angle =;        
	shoot->info.cfg_rx_info.base_cfg_info.reload_speed =;    
	shoot->info.cfg_rx_info.base_cfg_info.repeat_shot_mode =
	shoot->info.cfg_rx_info.base_cfg_info.repeat_shot_period =;;
	shoot->info.cfg_rx_info.base_cfg_info.state_work_time_max =;
	shoot->info.cfg_rx_info.base_cfg_info.speed_stop_mode =;         
	
	shoot->info.cfg_rx_info.base_cfg_info.stop_angle_err_max =;
	                                                                                     
	//拨盘复位堵转配置                            
	shoot->info.cfg_rx_info.reset_speed_block_cfg_info.block_judge_type =;
	                                      
	shoot->info.cfg_rx_info.reset_speed_block_cfg_info.speed_max =;
	shoot->info.cfg_rx_info.reset_speed_block_cfg_info.current_min =;
	shoot->info.cfg_rx_info.reset_speed_block_cfg_info.block_time_max =;
	
	shoot->info.cfg_rx_info.reset_speed_block_cfg_info.integral_value =;
	
  //拨盘速度环堵转配置
	shoot->info.cfg_rx_info.speed_block_cfg_info.block_judge_type =;
	
	shoot->info.cfg_rx_info.speed_block_cfg_info.speed_max = ;
	shoot->info.cfg_rx_info.speed_block_cfg_info.current_min =;
	shoot->info.cfg_rx_info.speed_block_cfg_info.block_time_max =;
	
	shoot->info.cfg_rx_info.speed_block_cfg_info.integral_value =;
	
	//拨盘角度环堵转配置
	
	shoot->info.cfg_rx_info.angle_block_cfg_info.block_judge_type =;
	                                             
	shoot->info.cfg_rx_info.angle_block_cfg_info.speed_max = ;
	shoot->info.cfg_rx_info.angle_block_cfg_info.current_min =;
	shoot->info.cfg_rx_info.angle_block_cfg_info.block_time_max =;
	                                             
	shoot->info.cfg_rx_info.angle_block_cfg_info.integral_value =;
	
	
	#if DIAL_IS_ANSOLUTE_ANGLE
	  
		//拨盘基本配置
		shoot->info.cfg_rx_info.base_cfg_info.reset_offist_angle =; 
		//拨盘复位堵转配置
		shoot->info.cfg_rx_info.reset_speed_block_cfg_info.angle_err_integral_max =;
		//拨盘速度环堵转配置
		shoot->info.cfg_rx_info.speed_block_cfg_info.angle_err_integral =;
		//拨盘角度环堵转配置
		shoot->info.cfg_rx_info.angle_block_cfg_info.angle_err_integral =;
		
		Absolute_Angle_Target_Init(shoot);                         //初始化绝对角度角度环目标角度
	#else
		//拨盘基本配置
		shoot->info.cfg_rx_info.base_cfg_info.reset_speed =;               
	  shoot->info.cfg_rx_info.base_cfg_info.reset_adjust_angle =;        
	  shoot->info.cfg_rx_info.base_cfg_info.reset_speed_work_time_max =; 
		//拨盘复位堵转配置
		shoot->info.cfg_rx_info.reset_speed_block_cfg_info.angle_sum_err_integral_max =;
		//拨盘速度环堵转配置
		shoot->info.cfg_rx_info.speed_block_cfg_info.angle_sum_err_integral_max =;
		//拨盘角度环堵转配置
		shoot->info.cfg_rx_info.angle_block_cfg_info.angle_sum_err_integral_max =;
		
	#endif
  
	//*/
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
		                                                          && shoot->cmd.vision_tx_cmd.is_ready_flag == 1)  //可以立即打弹情况下才能初始化
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
			#if DIAL_IS_ANSOLUTE_ANGLE
			  shoot->cmd.dial_tx_cmd.angle_target = shoot->misc.behind_absolute_angle_target;
			#else
			  shoot->cmd.dial_tx_cmd.angle_sum_target -= shoot->misc.beyond_angle;
			#endif
			
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
				#if DIAL_IS_ANSOLUTE_ANGLE
			    shoot->cmd.dial_tx_cmd.angle_target = shoot->misc.behind_absolute_angle_target;
			  #else
			    shoot->cmd.dial_tx_cmd.angle_sum_target -= shoot->misc.beyond_angle;
			  #endif
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
	if(cfg_info->block_judge_type == 0)   //堵转判断方法1，根据速度，电流，堵转时间判断
	{
		if(abs(rt_info->speed) < cfg_info->speed_max && abs(rt_info->current) > cfg_info->current_min)
    {
		  if(cfg_info->block_time >= cfg_info->block_time_max)
		  {
		  	flag = 1;
				cfg_info->block_time = 0;               //自动清零
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
	
	else if(cfg_info->block_judge_type == 1 && cmd->mode == DIAL_ANGLE)    //堵转判断方法2，只适用于角度环，根据角度误差积分判断
	{
		
		#if DIAL_IS_ANSOLUTE_ANGLE                         //绝对角度
		  if(abs(Absolute_Angle_Err_Calculate(cmd->angle_target, rt_info->angle, rt_info->current)) > shoot.info.cfg_rx_info.base_cfg_info.stop_angle_err_max)
		  {
		    cfg_info->angle_err_integral += cfg_info->integral_value * Absolute_Angle_Err_Calculate(cmd->angle_target, rt_info->angle, rt_info->current);
		    if(cfg_info->angle_err_integral >= cfg_info->angle_err_integral_max) 
		    {
		  	  flag = 1;
			    cfg_info->angle_err_integral = 0;              //积分清零
		    }
		  }
		  else
		  {
			  cfg_info->angle_err_integral = 0;
			}	
			
			
		#else                                   //相对角度
		  if(abs(cmd->angle_sum_target - misc->angle_sum) > shoot.info.cfg_rx_info.base_cfg_info.stop_angle_err_max)
			{
				cfg_info->angle_sum_err_integral += cfg_info->integral_value * (cmd->angle_sum_target - misc->angle_sum);
		    if(abs(cfg_info->angle_sum_err_integral) >= cfg_info->angle_sum_err_integral_max)
		    {
			    flag = 1;
		  	  cfg_info->angle_sum_err_integral = 0;        //积分清零
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


/**
 * @brief   拨盘实时状态更新
 * @note    elec_level_flag外部文件更新
 */
void Dial_Work_State_Update(Shoot_t* shoot)
{
	static uint8_t last_elec_level_flag;                           //保存上一个电平标志位，用于检测电平变化
	static uint16_t work_time;                                     //记录工作时间，想看状态变化可以将其提升至全局变量
	static uint16_t now_tick;                                      //保存当前tick值
	static uint16_t last_tick;                                     //保存上一次tick值
	static DIAL_ANGLE_DATA_TYPE         block_memory_angle;        //保存绝对角度前提下补弹堵转时拨盘后面的角度环目标值,可放misc
	static DIAL_ANGLE_SUM_DATA_TYPE     block_memory_angle_sum;    //保存相对角度前提下补弹堵转时拨盘后面的角度环目标值,可放misc
	 
	switch (shoot->cmd.dial_tx_cmd.work_state)
	{
		case SLEEP:                                              //睡眠模式更新，只卸力
			work_time = 0;
		  shoot->cmd.dial_tx_cmd.current_target = 0;
		  //堵转判断中时间，积分全部清零
		  shoot->info.cfg_rx_info.reset_speed_block_cfg_info.block_time = 0;
		  shoot->info.cfg_rx_info.reset_speed_block_cfg_info.angle_err_integral = 0;
		  shoot->info.cfg_rx_info.reset_speed_block_cfg_info.angle_sum_err_integral = 0;
		
		  shoot->info.cfg_rx_info.speed_block_cfg_info.block_time = 0;
		  shoot->info.cfg_rx_info.speed_block_cfg_info.angle_err_integral = 0;
		  shoot->info.cfg_rx_info.speed_block_cfg_info.angle_sum_err_integral = 0;
		  
		  shoot->info.cfg_rx_info.angle_block_cfg_info.block_time= 0;
		  shoot->info.cfg_rx_info.angle_block_cfg_info.angle_err_integral = 0;
		  shoot->info.cfg_rx_info.angle_block_cfg_info.angle_sum_err_integral = 0;
		
		  if(shoot->info.rt_rx_info.flag_Info.is_sleep_flag == 0 && shoot->flag.init_flag == 0 
		                                                          && shoot->cmd.vision_tx_cmd.is_ready_flag == 1)
			{
		     shoot->cmd.dial_tx_cmd.work_state = RESETING;                 //拨盘进入复位状态
		    //绝对角度前提下复位只用角度环到零点角度即可
		    #if  DIAL_IS_ANSOLUTE_ANGLE
		      shoot->cmd.dial_tx_cmd.mode = DIAL_ANGLE;         
			    shoot->cmd.dial_tx_cmd.angle_target = shoot->info.cfg_rx_info.base_cfg_info.reset_angle;
				
		    //相对角度前提下需要先速度环找限位，再角度环调整最佳打弹角度
		    #else
			     shoot->cmd.dial_tx_cmd.mode = DIAL_SPEED;          
					 //宏定义用于变换速度方向，方向取决于拨盘正转使弹丸触碰限位还是反转触碰，正转是碰到枪管限位	 
		       shoot->cmd.dial_tx_cmd.speed_target = -DIAL_MEC_LIMIT * shoot->info.cfg_rx_info.base_cfg_info.reset_speed;  
	      #endif
			} 
			
		  break;
		
		case RESETING:                                            //复位状态更新
			work_time ++;
		  
		#if DIAL_IS_ANSOLUTE_ANGLE == 0              
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
				  //记录拨盘的起始角度和，用于后续计算超出角度
				  shoot->misc.angle_sum_start = shoot->misc.angle_sum;     
        }  			
				
			}
			#endif
			
			//这一部分，绝对相对角度复位都需要，相对角度只是多了速度环找限位的过程
			if(DIAL_IS_ANSOLUTE_ANGLE == 1 || (DIAL_IS_ANSOLUTE_ANGLE == 0 && shoot->flag.reset_speed_flag == 1))
			{
				if(Dial_Block_Check(&shoot->info.rt_rx_info.dial_info,&shoot->misc,
				                  &shoot->info.cfg_rx_info.angle_block_cfg_info,&shoot->cmd.dial_tx_cmd) == 1)   
			  {
			  	shoot->cmd.dial_tx_cmd.work_state = WAITING;
				  shoot->flag.init_flag = 1;
				  //记录拨盘的起始角度和，用于后续计算相对角度超出角度
				  shoot->misc.angle_sum_start = shoot->misc.angle_sum;     
			  }
			  else if(work_time >= shoot->info.cfg_rx_info.base_cfg_info.reset_angle_work_time_max)
			  {
				  shoot->cmd.dial_tx_cmd.work_state = WAITING;
				  shoot->flag.init_flag = 1;
				  //记录拨盘的起始角度和，用于后续计算相对角度超出角度
				  shoot->misc.angle_sum_start = shoot->misc.angle_sum;     
			  }
			}
		  
			//初始化完成，切换进等待模式
			if(shoot->flag.init_flag == 0 && ((abs(shoot->cmd.dial_tx_cmd.angle_sum_target - shoot->misc.angle_sum) 
			                              <= shoot->info.cfg_rx_info.base_cfg_info.stop_angle_err_max)
			                              || abs(Half_Cir_Handle(shoot->cmd.dial_tx_cmd.angle_target - shoot->info.rt_rx_info.dial_info.angle)) 
		                                <= shoot->info.cfg_rx_info.base_cfg_info.stop_angle_err_max))
			{                                
				shoot->cmd.dial_tx_cmd.work_state = WAITING;
				shoot->cmd.dial_tx_cmd.mode = DIAL_ANGLE;
				shoot->flag.init_flag = 1;
				
				//记录拨盘的起始角度和，用于后续计算相对角度超出角度
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
			
			//需要同时符合没有电机掉线，可以立即开火，开火操作和当前开火模式对应的开火操作相同，才能补弹
			if(shoot->info.rt_rx_info.flag_Info.is_mtr_offline_flag == 0)
			{
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
			}
			else
      {
				shoot->cmd.vision_tx_cmd.is_ready_flag = 0;
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
				  	  Angle_Target_Switch(shoot);
					    work_time ++;
					    last_tick = now_tick;                              //实时更新tick
				    }
				    //拨盘在周期内完成角度环并停下
				    else if(now_tick - last_tick < shoot->info.cfg_rx_info.base_cfg_info.repeat_shot_period 
				 	                               && (abs(shoot->cmd.dial_tx_cmd.angle_sum_target - shoot->misc.angle_sum) 
						                             <= shoot->info.cfg_rx_info.base_cfg_info.stop_angle_err_max
						                             || Half_Cir_Handle(shoot->cmd.dial_tx_cmd.angle_target - shoot->info.rt_rx_info.dial_info.angle) 
						                             <= shoot->info.cfg_rx_info.base_cfg_info.stop_angle_err_max))
				    {
					    work_time = 0;
						
            }
			  	}					
				
				   //实时更新拨弹进度
					#if DIAL_IS_ANSOLUTE_ANGLE
					shoot->cmd.vision_tx_cmd.reload_sche =shoot->cmd.vision_tx_cmd.reload_sche = (float)(shoot->info.rt_rx_info.dial_info.angle - shoot->misc.behind_absolute_angle_target) 
							                         / (float)shoot->info.cfg_rx_info.base_cfg_info.oneshot_angle;	
					#else
				  shoot->cmd.vision_tx_cmd.reload_sche = (float)(shoot->cmd.dial_tx_cmd.angle_sum_target - shoot->misc.angle_sum) 
		                                   / (float)shoot->info.cfg_rx_info.base_cfg_info.oneshot_angle;	
					#endif
					
				
				  //堵转处理切退弹模式
				  if(Dial_Block_Check(&shoot->info.rt_rx_info.dial_info,&shoot->misc,&shoot->info.cfg_rx_info.angle_block_cfg_info,&shoot->cmd.dial_tx_cmd) == 1)
			    {
			  	  shoot->cmd.dial_tx_cmd.work_state = RECOIL;
						
						shoot->cmd.dial_tx_cmd.mode = DIAL_ANGLE;
						
						//储存转换前当前位置后面的角度环目标值，用于后面弥补
						#if DIAL_IS_ANSOLUTE_ANGLE
						  block_memory_angle = shoot->misc.behind_absolute_angle_target;
						#else
						  block_memory_angle_sum = shoot->cmd.dial_tx_cmd.angle_sum_target - shoot->info.cfg_rx_info.base_cfg_info.oneshot_angle;
						#endif
						
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
						
				  	shoot->cmd.vision_tx_cmd.is_ready_flag = 1;

			    }
				  //拨盘完成角度环，停下来才切换等待模式
				  else if(shoot->misc.angle_sum <= shoot->info.cfg_rx_info.base_cfg_info.stop_angle_err_max)
				  {
					  shoot->cmd.dial_tx_cmd.work_state = WAITING;
					  shoot->cmd.dial_tx_cmd.mode = DIAL_ANGLE;
					  work_time = 0;
					  
					  shoot->cmd.vision_tx_cmd.is_ready_flag = 1;
						
				  }
		    
	        break;
				
		    case DIAL_SPEED:
				
			    if(shoot->info.rt_rx_info.flag_Info.elec_level_flag == 1)
				  {
						
						#if DIAL_IS_ANSOLUTE_ANGLE 
						  //计算绝对角度当前角度超出后面角度环目标值的角度
						  shoot->misc.beyond_angle =shoot->info.rt_rx_info.dial_info.angle - shoot->misc.behind_absolute_angle_target;
						  shoot->cmd.vision_tx_cmd.reload_sche = (float)(shoot->info.cfg_rx_info.base_cfg_info.oneshot_angle -shoot->misc.beyond_angle)
                               						/ (float)shoot->info.cfg_rx_info.base_cfg_info.oneshot_angle;	
						#else
						  //计算相对角度的当前角度超出后面角度环目标值的角度
				      shoot->misc.beyond_angle = (shoot->misc.angle_sum - shoot->misc.angle_sum_start)
				                                                         % shoot->info.cfg_rx_info.base_cfg_info.oneshot_angle;
				      //速度环与角度环计算方式稍微不同，也可尝试用同一种方法
				      shoot->cmd.vision_tx_cmd.reload_sche = (float)(shoot->info.cfg_rx_info.base_cfg_info.oneshot_angle - shoot->misc.beyond_angle) 
		                                      / (float)shoot->info.cfg_rx_info.base_cfg_info.oneshot_angle;	
						  //实时更新角度和目标值
				      shoot->cmd.dial_tx_cmd.angle_sum_target = shoot->misc.angle_sum;
						#endif
						
				    //堵转处理
			      if(Dial_Block_Check(&shoot->info.rt_rx_info.dial_info,NULL,&shoot->info.cfg_rx_info.speed_block_cfg_info,&shoot->cmd.dial_tx_cmd) == 1)
			      {
			        shoot->cmd.dial_tx_cmd.work_state = RECOIL;
				      shoot->cmd.dial_tx_cmd.mode = DIAL_ANGLE;
				      
						  //储存转换前当前位置后面的角度环目标值，用于后面弥补
						  #if DIAL_IS_ANSOLUTE_ANGLE
						    block_memory_angle = shoot->misc.behind_absolute_angle_target;
						  #else
						    block_memory_angle_sum = shoot->cmd.dial_tx_cmd.angle_sum_target - shoot->info.cfg_rx_info.base_cfg_info.oneshot_angle;
						  #endif
							
							Angle_Target_Switch(shoot);                              //堵转也要调整一弹丸角度，堵转处理后会补回来
							
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
								#if DIAL_IS_ABSOLUTE_ANGLE
							    shoot->cmd.dial_tx_cmd.angle_target = shoot->misc.front_absolute_angle_target;
							  #else
							    shoot->cmd.dial_tx_cmd.angle_sum_target +=(shoot->info.cfg_rx_info.base_cfg_info.oneshot_angle - shoot->misc.beyond_angle);
							  #endif
							  break;
							
							case BACKWARD:             //往后转动，达到后面的角度环集合目标值
								#if DIAL_IS_ABSOLUTE_ANGLE
							    shoot->cmd.dial_tx_cmd.angle_target = shoot->misc.behind_absolute_angle_target;
							  #else
							    shoot->cmd.dial_tx_cmd.angle_sum_target -= shoot->misc.beyond_angle;
							  #endif
								
							  break;
							
							case ROUNDING_UP:          //四舍五入，距离哪边近就去哪边
								
								if(2 * shoot->misc.beyond_angle > shoot->info.cfg_rx_info.base_cfg_info.oneshot_angle)
								{
									#if DIAL_IS_ABSOLUTE_ANGLE
									  shoot->cmd.dial_tx_cmd.angle_target = shoot->misc.front_absolute_angle_target;
							    #else
							      shoot->cmd.dial_tx_cmd.angle_sum_target +=(shoot->info.cfg_rx_info.base_cfg_info.oneshot_angle - shoot->misc.beyond_angle);
							    #endif
								}
								else if(2 * shoot->misc.beyond_angle <= shoot->info.cfg_rx_info.base_cfg_info.oneshot_angle)
								{
									#if DIAL_IS_ABSOLUTE_ANGLE
									  shoot->cmd.dial_tx_cmd.angle_target = shoot->misc.behind_absolute_angle_target;
							    #else
						      	shoot->cmd.dial_tx_cmd.angle_sum_target -= shoot->misc.beyond_angle;
							    #endif
								}
								break;
						}
				  }
			    break;
			 }
				  
		   break;	
				 
			 
		/*堵转处理逻辑：堵转时并非往后退到后面的角度环目标角度，而是先往后退一颗弹丸的角度
			 ，成功退回后再往前补到前面的角度环的目标角度，成功后进入等待状态，任何超时均进入等待状态	*/ 
		case RECOIL:                                                           //堵转状态更新
			
		  //成功退弹
			if(abs(shoot->misc.angle_sum - shoot->cmd.dial_tx_cmd.angle_sum_target) <= shoot->info.cfg_rx_info.base_cfg_info.stop_angle_err_max
				|| abs(Half_Cir_Handle(shoot->cmd.dial_tx_cmd.angle_target - shoot->info.rt_rx_info.dial_info.angle)) 
		    <= shoot->info.cfg_rx_info.base_cfg_info.stop_angle_err_max)
			{
				shoot->flag.dial_block_flag = 0;
				work_time = 0; 
				
				//弥补堵转处理时退一颗弹而造成的角度差
				#if DIAL_IS_ANSOLUTE_ANGLE
					shoot->cmd.dial_tx_cmd.angle_target = block_memory_angle;
				#else
					shoot->cmd.dial_tx_cmd.angle_sum_target = block_memory_angle_sum;
				#endif
				
				if(abs(shoot->misc.angle_sum - shoot->cmd.dial_tx_cmd.angle_sum_target) <= shoot->info.cfg_rx_info.base_cfg_info.stop_angle_err_max
				|| abs(Half_Cir_Handle(shoot->cmd.dial_tx_cmd.angle_target - shoot->info.rt_rx_info.dial_info.angle)) 
		    <= shoot->info.cfg_rx_info.base_cfg_info.stop_angle_err_max)
				{
					shoot->cmd.dial_tx_cmd.work_state = WAITING;
				  shoot->cmd.dial_tx_cmd.mode = DIAL_ANGLE;
					shoot->cmd.vision_tx_cmd.is_ready_flag = 1;
					work_time = 0;
				}
				//超时退出
		    else if(work_time >= shoot->info.cfg_rx_info.base_cfg_info.state_work_time_max)
			  {
				  shoot->cmd.dial_tx_cmd.work_state = WAITING;
				  shoot->cmd.dial_tx_cmd.mode = DIAL_ANGLE;
				  shoot->flag.dial_block_flag = 0;
					shoot->cmd.vision_tx_cmd.is_ready_flag = 1;
				  work_time = 0; 
		   	}
				else
				{
					work_time ++; 
				
				}
				
			}
			//超时退出
		  else if(work_time >= shoot->info.cfg_rx_info.base_cfg_info.state_work_time_max)
			{
				shoot->cmd.dial_tx_cmd.work_state = WAITING;
				shoot->cmd.dial_tx_cmd.mode = DIAL_ANGLE;
				shoot->flag.dial_block_flag = 0;
				shoot->cmd.vision_tx_cmd.is_ready_flag = 1;
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

	Angle_Sum_Calculate(shoot);
	Absolute_Angle_Target_Transfor(shoot);
	
	Shoot_Work_State_Update(shoot);    //更新发射机构工作状态
	Shoot_Mode_Update(shoot);          //更新发射机构模式
	Dial_Work_State_Update(shoot);  //拨盘复位状态
	
}


