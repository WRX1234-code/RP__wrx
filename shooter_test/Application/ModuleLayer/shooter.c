#include "shooter.h"
#include "motor.h"
#include "rc_sensor.h"
#include "config_uart.h"
#include "judge_protocol.h"

//Shoot_t shoot={
//	.dial={
//		.dial_config=&Dial,
//	  .dial_work_state=DIAL_SLEEP,
//		.dial_speed_target=DIAL_RELOAD_SPEED,
//	  .dial_work_time=0,
//		.dial_angle_sum=0,
//	},
//	.fric={
//		.thr_fric[FRIC_UP]=&Fric_Up,
//		.thr_fric[FRIC_R]=&Fric_R,
//		.thr_fric[FRIC_L]=&Fric_L,
//	  .fric_speed_target=FRIC_SPEED_TARGET,
//	},
//	.pitch={
//		.pitch_motor=&Pitch,
//		
//	  .p_k=0.0007f,
//	},
//	.shoot_safe_state=locked,
//	.block_flag=0,
//	.fire_flag=0,
//	.fric_ok_flag=1,
//	.shoot_safe_cnt=0,
//	.shoot_load_state=LOAD_OK,

//};

Shoot_t shoot;


void Shoot_Init(Shoot_t *shoot)
{
	shoot->fric.thr_fric[FRIC_UP]=RM_Group.motor[0];
	shoot->fric.thr_fric[FRIC_R]=RM_Group.motor[1];
	shoot->fric.thr_fric[FRIC_L]=RM_Group.motor[2];
	shoot->dial.dial_config=RM_Group.motor[3];

	shoot->pitch.pitch_motor=&Pitch;
	shoot->pitch.p_k=0.0007f;
	shoot->fric.fric_speed_target=0;

	shoot->shoot_safe_state=locked;
//	shoot->awake_flag=1;                            //reset
	shoot->block_flag=0;
	shoot->fire_flag=0;
	shoot->fric_ok_flag=1;
	shoot->shoot_safe_cnt=0;
	shoot->dial.dial_angle_sum=0;
	shoot->dial.dial_work_time=0;
	shoot->shoot_load_state=LOAD_OK;
	
	shoot->dial.dial_zero_angle_sum=shoot->dial.dial_config->rx_info->encoder_sum; //reset(when reset,delete)
	
	shoot->k=0.3;
	
	shoot->pitch.pitch_motor->ctrl->angle_ctrl_outer->target=3250.f;
	
	judge.start_burst_flag=1;
	shoot->fric_speed=9200.f;
	shoot->dial_speed=2000.f;
}

void Shoot_Safe_State_Update(Shoot_t *shoot)
{
	static float roller_step;
	static float last_roller_step;
	
	roller_step=rc_sensor.info->thumbwheel.step[0];
	
	if((roller_step!=last_roller_step)||rc_sensor.info->V.status==release_to_press)
	{
		shoot->shoot_safe_cnt++;
	}
	
	if(shoot_heart_cnt>=70)
	{
		shoot->shoot_safe_state=locked;
		shoot->shoot_safe_cnt=0;
		shoot->fric.fric_speed_target=0;
	}
	else if(shoot->shoot_safe_cnt%2==0&&shoot_heart_cnt<70)
	{
		shoot->shoot_safe_state=locked;
		shoot->fric.fric_speed_target=0;
	}
	else if(shoot->shoot_safe_cnt%2==1&&shoot_heart_cnt<70)
	{
		shoot->shoot_safe_state=unlock;
		shoot->shoot_work_state=CEASEFIRE;
    shoot->fric.fric_speed_target=shoot->fric_speed;//FRIC_SPEED_TARGET
//		if(shoot->awake_flag==1)                          //      
//		{                                                 //   
//			shoot->dial.dial_work_state=DIAL_AWAKE;         //       reset     
//		}                                                 //
	}                       
	
	last_roller_step=roller_step;
}


void Shoot_Work_State_Update(Shoot_t *shoot)
{
	static float value;
	static float last_value;
	static float roller_value;
	static float roller_step;
	static float last_roller_step;
//	static uint8_t key_cnt;
	
	value=rc_sensor.info->s2.value;
//  last_value=0x03;
	roller_value=rc_sensor.info->thumbwheel.value;
	roller_step=rc_sensor.info->thumbwheel.step[2];
	
//  if(communicate_control_mode==RC_MODE)
//	{
		if(value==0x03&&(last_value==0x01||last_value==0x03))
	  {
		  shoot->shoot_work_state=SIMGLE_SHOT;
	  }
	  else if(value==0x01&&(last_value==0x03||last_value==0x01))
	  {
		  shoot->shoot_work_state=BURST;
	  }
	  else
	  {
	  	shoot->shoot_work_state=CEASEFIRE;
	  }
		
		last_value=value;
//	}
//	else if(communicate_control_mode==KEY_MODE)
//	{
//		if(rc_sensor.info->B.status==release_to_press)
//		{
//			key_cnt++;
//		}
//		
//		if(key_cnt%2==0)
//		{
//			shoot->shoot_work_state=SIMGLE_SHOT;
//		}
//		else if(key_cnt%2==1)
//		{
//			shoot->shoot_work_state=BURST;
//		}
//	
//	}
	
	
	switch (shoot->shoot_work_state)
	{
		case CEASEFIRE:
			shoot->fire_flag=0;
		  shoot->firing_flag=0;
		  shoot->dial.dial_work_state=DIAL_SLEEP;
		
			break;
		
		case SIMGLE_SHOT:
			shoot->firing_flag=0;
			if(roller_step!=last_roller_step)
			{
				shoot->fire_flag=1;
				shoot->shoot_load_state=LOAD_NO;
				shoot->dial.dial_mode=DIAL_ANGLE;
				
				judge.shoot_mode=0;
				
				Shooting_Cmd_Excute_Tick_Calculating(0);
			}
			
			last_roller_step=roller_step;
			
			break;
		
		case BURST:
			shoot->fire_flag=0;
			if(roller_step==last_roller_step&&roller_value>0)
			{
				shoot->firing_flag=1;
				shoot->shoot_load_state=LOAD_NO;
				shoot->dial.dial_mode=DIAL_SPEED;
				
				judge.shoot_mode=1;
				if(judge.start_burst_flag==1)
				{
					Shooting_Cmd_Excute_Tick_Calculating(0);
				  judge.start_burst_flag=0;
				}
								
			}
			else
			{
				shoot->firing_flag=0;
				
				judge.shoot_mode=0;
				judge.start_burst_flag=1;
			}
			last_roller_step=roller_step;
			
		  break;
		default:
		  break;	
	}	
}


uint8_t Motor_Stuck_Check(Motor_RM_t* motor,uint16_t speed,uint16_t current,uint16_t stuck_time)
{

	if(abs(motor->rx_info->encoder_speed)<speed&&abs(motor->rx_info->torque_current_raw)>current)
	{
		shoot.block_time++;
	}
	
	if(shoot.block_time>=stuck_time)
	{
		
		shoot.block_time=stuck_time;
		return 1;
	}
	else{
	  return 0;
	}
	
}

void Shoot_Reload(Shoot_t* shoot)
{
	
	switch (shoot->dial.dial_work_state)
	{
//		case DIAL_AWAKE:                                                                        //      
//			shoot->dial.dial_mode=DIAL_SPEED;                                                     //        
//		  shoot->dial.dial_speed_target=-DIAL_RELOAD_SPEED;                                     //           
//		  if(Motor_Stuck_Check(shoot->dial.dial_config,30,1000,20)==1)                          //        
//			{                                                                                     //      
//				shoot->dial.dial_work_time=0;                                                       //   
//				shoot->dial.dial_speed_target=0;                                                    //      reset
//				shoot->dial.dial_work_state=DIAL_SLEEP;                                             //     
//				shoot->awake_flag=0;                                                                // 
//				shoot->block_time=0;                                                                //  
//				shoot->dial.dial_zero_angle_sum=shoot->dial.dial_config->rx_info->encoder_sum;      //   
//				shoot->dial.dial_angle_sum=shoot->dial.dial_config->rx_info->encoder_sum;           //
//			}                                                                                     //    
//			break;                                                                                //   
		 
		case DIAL_SLEEP:     
			shoot->dial.dial_work_time=0;
			shoot->dial.dial_speed_target=0;
			if(shoot->shoot_load_state==LOAD_NO&&(shoot->fire_flag==1||shoot->firing_flag==1)&&shoot->block_flag==0)//&&shoot->fric_ok_flag==1
			{
				shoot->dial.dial_work_state=DIAL_RELOAD;
				if(shoot->shoot_work_state==SIMGLE_SHOT)
				{
					shoot->dial.dial_angle_sum+=ONESHOT_ANGLE;
				
				}
				
			}
		
			break;
		
		case DIAL_RELOAD:
			
			switch (shoot->dial.dial_mode)
			{
				case DIAL_SPEED:
					if(shoot->firing_flag==1)
					{
						shoot->dial.dial_speed_target=shoot->dial_speed;//DIAL_RELOAD_SPEED
				    shoot->dial.dial_angle_sum=shoot->dial.dial_config->rx_info->encoder_sum;
//						shoot->dial.zero_start_reload_cnt=(shoot->dial.dial_angle_sum-shoot->dial.dial_zero_angle_sum)/ONESHOT_ANGLE;
						shoot->dial.extra_angle=(shoot->dial.dial_angle_sum-shoot->dial.dial_zero_angle_sum)%ONESHOT_ANGLE;
			  	  if(Motor_Stuck_Check(shoot->dial.dial_config,30,2000,100)==1)
			      {
				      shoot->dial.dial_work_state=DIAL_RECOIL;
							shoot->dial.dial_angle_sum-=(ONESHOT_ANGLE+shoot->dial.extra_angle);
							
						  shoot->dial.dial_mode=DIAL_ANGLE;
				      shoot->block_flag=1;
				      shoot->dial.dial_work_time=0;
//						  shoot->firing_flag=0;
			      }
//			      else if(shoot->fric_ok_flag==0||shoot->dial.dial_work_time>DIAL_WORK_TIME_MAX)   
//			      {
//				      shoot->dial.dial_speed_target=0;
//		  	      shoot->dial.dial_work_state=DIAL_SLEEP;
//				      shoot->dial.dial_work_time=0;
//				      shoot->shoot_load_state=LOAD_OK;
//					  }  
//				    else
//						{
//							shoot->dial.dial_work_time++;
//						} 
					}
					else if(shoot->firing_flag==0)
					{
						shoot->dial.dial_speed_target=0;
						shoot->shoot_load_state=LOAD_OK;
						shoot->block_time=0;
						shoot->dial.dial_work_time=0;
						shoot->dial.dial_mode=DIAL_ANGLE;
//						shoot->dial.dial_angle_sum-=shoot->dial.extra_angle;
					}
					
			  	break;
				
				case DIAL_ANGLE:
					
			    if(Motor_Stuck_Check(shoot->dial.dial_config,30,1000,100)==1)
			    {
				    shoot->dial.dial_work_state=DIAL_RECOIL;
						shoot->dial.dial_angle_sum-=2*ONESHOT_ANGLE;
						shoot->dial.dial_mode=DIAL_ANGLE;
				    shoot->block_flag=1;
				    shoot->dial.dial_work_time=0;
						shoot->fire_flag=0;
			    }
//			    else if(shoot->fric_ok_flag==0||shoot->dial.dial_work_time>DIAL_WORK_TIME_MAX)
//			    {
//				    shoot->dial.dial_speed_target=0;
//		  	    shoot->dial.dial_work_state=DIAL_SLEEP;
//				    shoot->dial.dial_work_time=0;
//				    shoot->shoot_load_state=LOAD_OK;
//						shoot->fire_flag=0;
//			    }
					else if(shoot->dial.dial_config->rx_info->encoder_speed==0&&shoot->dial.dial_config->rx_info->torque_current_raw<200)
					{
						shoot->fire_flag=0;
						shoot->block_time=0;
						shoot->dial.dial_work_time=0;
						shoot->shoot_load_state=LOAD_OK;
					  shoot->dial.dial_work_state=DIAL_SLEEP;
				  }
//				  else
//			    {
//				      shoot->dial.dial_work_time++;
//			    }
				  break;
		  }
			
			break;
		
		case DIAL_RECOIL:
			shoot->dial.dial_mode=DIAL_ANGLE;
		  shoot->block_time=0;
			if(shoot->dial.dial_config->rx_info->encoder_sum<=shoot->dial.dial_angle_sum)
			{
				shoot->dial.dial_work_time=0;
				shoot->dial.dial_speed_target=0;
				shoot->shoot_load_state=LOAD_OK;
				shoot->block_flag=0;
				shoot->dial.dial_work_state=DIAL_SLEEP;
				
			}
//			else if(shoot->dial.dial_work_time>DIAL_WORK_TIME_MAX)
//			{
//				shoot->dial.dial_work_time=0;
//				shoot->dial.dial_speed_target=0;
//				shoot->shoot_load_state=LOAD_OK;
//				shoot->block_flag=0;
//				shoot->dial.dial_work_state=DIAL_SLEEP;
//			}
//			else
//			{
//				shoot->dial.dial_work_time++;
//			}
				
			break;
		default:
		break;
	}
}

void Remote_receive(Shoot_t *shoot)
{
	shoot->pitch.pitch_motor->ctrl->angle_ctrl_outer->target-=rc_sensor.info->ch1*shoot->pitch.p_k;
	
	if(shoot->pitch.pitch_motor->ctrl->angle_ctrl_outer->target>=3965.f)
	{
		shoot->pitch.pitch_motor->ctrl->angle_ctrl_outer->target=3965.f;
	}
	else if(shoot->pitch.pitch_motor->ctrl->angle_ctrl_outer->target<=2660.f)
	{
		shoot->pitch.pitch_motor->ctrl->angle_ctrl_outer->target=2660.f;
	}
}


void Shoot_PID_Calculate(Shoot_t *shoot)
{
	uint8_t i;
	float k=0.0067f;	
	float add_err;
	float add_I;
	float out;
	if(shoot->dial.dial_mode==DIAL_SPEED)
	{
		shoot->dial.dial_config->ctrl->speed_ctrl->target=shoot->dial.dial_speed_target;
	  shoot->dial.dial_config->ctrl->speed_ctrl->measure=shoot->dial.dial_config->rx_info->encoder_speed;
		
		add_err=shoot->dial.dial_config->ctrl->speed_ctrl->measure-shoot->last_speed;
		add_I=add_err*k;
		
		shoot->last_speed=shoot->dial.dial_config->ctrl->speed_ctrl->measure;
		
	  shoot->dial.dial_config->ctrl->speed_ctrl->err=shoot->dial.dial_config->ctrl->speed_ctrl->target-shoot->dial.dial_config->ctrl->speed_ctrl->measure;
		single_pid_ctrl(shoot->dial.dial_config->ctrl->speed_ctrl);
	
		out=shoot->dial.dial_config->ctrl->speed_ctrl->out+add_I;
		
		out=constrain(out,-9000,9000);
		shoot->dial.dial_config->tx_info->torque=out*shoot->k+shoot->last_speed_out*(1-shoot->k);
		
		shoot->last_speed_out=out;
	}
	else if(shoot->dial.dial_mode==DIAL_ANGLE)
	{
		
		shoot->dial.dial_config->ctrl->angle_ctrl_outer->target=shoot->dial.dial_angle_sum;
		shoot->dial.dial_config->ctrl->angle_ctrl_outer->measure=shoot->dial.dial_config->rx_info->encoder_sum;
		shoot->dial.dial_config->ctrl->angle_ctrl_outer->err=shoot->dial.dial_config->ctrl->angle_ctrl_outer->target-shoot->dial.dial_config->ctrl->angle_ctrl_outer->measure;
		
		single_pid_ctrl(shoot->dial.dial_config->ctrl->angle_ctrl_outer);
		
		shoot->dial.dial_config->ctrl->angle_ctrl_inner->target=shoot->dial.dial_config->ctrl->angle_ctrl_outer->out;
		shoot->dial.dial_config->ctrl->angle_ctrl_inner->measure=shoot->dial.dial_config->rx_info->encoder_speed;
		shoot->dial.dial_config->ctrl->angle_ctrl_inner->err=shoot->dial.dial_config->ctrl->angle_ctrl_inner->target-shoot->dial.dial_config->ctrl->angle_ctrl_inner->measure;
		
		single_pid_ctrl(shoot->dial.dial_config->ctrl->angle_ctrl_inner);
		shoot->dial.dial_config->tx_info->torque=shoot->dial.dial_config->ctrl->angle_ctrl_inner->out;
	}
	
	for(i=0;i<3;i++)
	{
		float k=-1;
		{
			if(i==2)
			{
				k=1;
			}
		}

		shoot->fric.thr_fric[i]->ctrl->speed_ctrl->target=k*shoot->fric.fric_speed_target;
		shoot->fric.thr_fric[i]->ctrl->speed_ctrl->measure=(float)shoot->fric.thr_fric[i]->rx_info->encoder_speed;
		shoot->fric.thr_fric[i]->ctrl->speed_ctrl->err=shoot->fric.thr_fric[i]->ctrl->speed_ctrl->target-shoot->fric.thr_fric[i]->ctrl->speed_ctrl->measure;
		single_pid_ctrl(shoot->fric.thr_fric[i]->ctrl->speed_ctrl);
		shoot->fric.thr_fric[i]->tx_info->torque=shoot->fric.thr_fric[i]->ctrl->speed_ctrl->out;
	}
	
	shoot->pitch.pitch_motor->ctrl->angle_ctrl_outer->measure=(float)shoot->pitch.pitch_motor->rx_info->encoder;
	shoot->pitch.pitch_motor->ctrl->angle_ctrl_outer->err=shoot->pitch.pitch_motor->ctrl->angle_ctrl_outer->target-shoot->pitch.pitch_motor->ctrl->angle_ctrl_outer->measure;
	single_pid_ctrl(shoot->pitch.pitch_motor->ctrl->angle_ctrl_outer);
	
	shoot->pitch.pitch_motor->ctrl->angle_ctrl_inner->target=shoot->pitch.pitch_motor->ctrl->angle_ctrl_outer->out;
	shoot->pitch.pitch_motor->ctrl->angle_ctrl_inner->measure=(float)shoot->pitch.pitch_motor->rx_info->encoder_speed;
	shoot->pitch.pitch_motor->ctrl->angle_ctrl_inner->err=shoot->pitch.pitch_motor->ctrl->angle_ctrl_inner->target-shoot->pitch.pitch_motor->ctrl->angle_ctrl_inner->measure;
	single_pid_ctrl(shoot->pitch.pitch_motor->ctrl->angle_ctrl_inner);
	
	shoot->pitch.pitch_motor->tx_info->torque=shoot->pitch.pitch_motor->ctrl->angle_ctrl_inner->out;
		
}

void Fric_State_Check(Shoot_t *shoot)
{
	
	for(uint8_t i=0;i<3;i++)
	{
			shoot->fric.fric_speed_fact[i]=shoot->fric.thr_fric[i]->rx_info->encoder_speed;
			shoot->fric.fric_speed_err[i]=abs(shoot->fric.fric_speed_fact[i]-shoot->fric.thr_fric[i]->ctrl->speed_ctrl->target);
			shoot->fric.fric_temperature_fact[i]=shoot->fric.thr_fric[i]->rx_info->temperature;
	}
	
	if(shoot->fric.fric_speed_err[FRIC_UP]<100&&shoot->fric.fric_speed_err[FRIC_R]<100&&shoot->fric.fric_speed_err[FRIC_L]<100)
	{
		shoot->fric_ok_flag=1;
	}
	if(abs(shoot->fric.fric_speed_fact[FRIC_UP])<1000||abs(shoot->fric.fric_speed_fact[FRIC_R])<1000||abs(shoot->fric.fric_speed_fact[FRIC_L])<1000)
	{
		shoot->fric_ok_flag=0;
	}
//	if(shoot->fric->fric_temperature_fact[M_Fric_B_UP]>shoot->fric->work_temperature_max||shoot->fric->fric_speed_fact[M_Fric_B_R]>shoot->fric->work_temperature_max||
//		 shoot->fric->fric_speed_fact[M_Fric_B_L]>shoot->fric->work_temperature_max||shoot->fric->fric_speed_fact[M_Fric_F_UP]>shoot->fric->work_temperature_max||
//		 shoot->fric->fric_speed_fact[M_Fric_F_R]>shoot->fric->work_temperature_max||shoot->fric->fric_speed_fact[M_Fric_F_L]>shoot->fric->work_temperature_max)
//	{
//		shoot->fric_ok_flag=0;
//	}
	
}


void Shooting_Fri_Speed_Adapt(Shoot_t* shoot)
{
	
/*用户定义参数**********************************************************/

#define SPEED_SAVE_NUM 2			  // 速度保存个数
	const float add_kp = 8.f;		  // 增加增益
	const float minus_kp = 8.f;		  // 减少增益
	
	const float over_blind_err = 0.2; // 超过多少内不调整
	
	const float less_blind_err = 0.2; // 低于多少内不调整
	const float max_adapt_range = 100; // 最大单次调整量

	/*函数变量**************************************************************/
	

	static float last_speed[SPEED_SAVE_NUM] = {0};					  // 保存上一发速度数组
	float now_speed =shoot_statistics.speed_now; // 当前速度

	uint8_t over_cnt = 0, less_cnt = 0;								  // 大于目标速度计数，小于目标速度计数
	
    /*执行弹速调整的条件*****************************************************/
	
	//超弹速！！！大量下降
	if(now_speed>25.f)
	{
		shoot->fric_speed-=40;
	
		return;
	}
	
	/*计算目前存储数组里弹速的情况******************************************/
	for (uint8_t i = 0; i < SPEED_SAVE_NUM; i++)
	{
		if (last_speed[i] == 0)
		{
			// 如果找到一个元素为零，跳出循环
			continue;
		}
		else if (last_speed[i]>24.9f)
		{
			over_cnt++;
		}
		else if (24.5f>last_speed[i])
		{
			less_cnt++;
		}
	}

	/*根据情况调整摩擦轮速度***********************************************/
	//施密特触发器
	if (now_speed -24.7f> over_blind_err) // 速度大于目标速度
	{
		if (over_cnt * minus_kp > max_adapt_range)//限幅
			return;
		shoot->fric_speed-= over_cnt * minus_kp;
	}
 
	else if (24.7f - now_speed > less_blind_err) // 速度小于目标速度
	{
		if (less_cnt * add_kp > max_adapt_range)
			return;
		if(less_cnt>=2)//数组里面两个都低于弹速才提高弹速
		{
			shoot->fric_speed+= less_cnt * add_kp;
		}
	}

	/*保存当前速度到数组**************************************************/
	for (uint8_t i = 1; i < SPEED_SAVE_NUM; i++)
	{
		last_speed[i] = last_speed[i - 1];
	}
	last_speed[0] = now_speed;
}
 

void Shoot_Sleep(Shoot_t *shoot)
{
	RM_Group.group_sleep(&RM_Group);
	
	shoot->dial.dial_angle_sum=shoot->dial.dial_config->rx_info->encoder_sum;
	
	shoot->pitch.pitch_motor->tx_info->torque=0;
	
}

void shoot_send(Shoot_t *shoot)
{
	RM_Group.group_set_torque(&RM_Group);
	
	shoot->pitch.pitch_motor->single_set_torque(shoot->pitch.pitch_motor);
	shoot_heart_cnt++;
}

void Shoot_Work(Shoot_t *shoot)
{
	Shoot_Safe_State_Update(shoot);
	switch (shoot->shoot_safe_state){
		case locked:
			Shoot_Sleep(shoot);
			
		  break;
		 
		case unlock:
			Remote_receive(shoot);
		 
			Shoot_Work_State_Update(shoot);
	   	Fric_State_Check(shoot);
		  Shoot_Reload(shoot);
		  Shooting_Fri_Speed_Adapt(shoot);
		  Shoot_PID_Calculate(shoot);
	    
		  break;
		
		default:
			break;
	}
	
	shoot_send(shoot);
	
}



