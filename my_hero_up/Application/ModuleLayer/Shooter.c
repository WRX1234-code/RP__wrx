#include "Shooter.h"
#include "motor.h"
#include "rc_sensor.h"
#include "Communicate.h"

Shoot_t shoot;

void Shoot_Init(Shoot_t *shoot)
{
	shoot->dial.dial_config=&W_Dail;
	shoot->fric.first_fric[M_Fric_B_UP]=RM_Group1.motor[0];
	shoot->fric.first_fric[M_Fric_B_R]=RM_Group1.motor[2];
	shoot->fric.first_fric[M_Fric_B_L]=RM_Group1.motor[3];

	shoot->fric.second_fric[M_Fric_F_UP-3]=RM_Group1.motor[1];
	shoot->fric.second_fric[M_Fric_F_R-3]=RM_Group2.motor[0];
	shoot->fric.second_fric[M_Fric_F_L-3]=RM_Group2.motor[2];
	
	shoot->shoot_safe_state=locked;
	shoot->block_flag=0;
	shoot->fire_flag=0;
	shoot->fric_ok_flag=0;
	shoot->shoot_safe_cnt=0;
	shoot->dial.dial_angle_sum=0;
	shoot->shoot_load_state=LOAD_OK;
	
}

void Shoot_Safe_State_Update(Shoot_t *shoot)
{
	static float roller_step;
	static float last_roller_step;
	
	roller_step=rc_sensor.info->thumbwheel.step[0];
	
	if(roller_step!=last_roller_step)
	{
		shoot->shoot_safe_cnt++;
	}
	
	if(shoot->shoot_safe_cnt%2==0||heart_cnt>=70)
	{
		shoot->shoot_safe_state=locked;
	}
	else if(shoot->shoot_safe_cnt%2==1&&heart_cnt<70)
	{
		shoot->shoot_safe_state=unlock;
		shoot->shoot_work_state=CEASEFIRE;

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
	
	value=rc_sensor.info->s2.value;
//  last_value=0x03;
	roller_value=rc_sensor.info->thumbwheel.value;
	roller_step=rc_sensor.info->thumbwheel.step[2];
	

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
			}
			else
			{
				shoot->firing_flag=0;
			}
			last_roller_step=roller_step;
			
		  break;
		default:
		  break;	
	}	
}
	
uint8_t Motor_Stuck_Check(Motor_RM_t* motor,uint16_t speed,int16_t current,uint16_t stuck_time)
{
	uint16_t time=0;
	uint8_t flag;
	if(abs(motor->rx_info->encoder_speed)<speed&&abs(motor->rx_info->torque_current)>current)
	{
		time++;
	}
	else
	{
	  flag=0;
	}
	
	if(time>=stuck_time)
	{
		flag=1;
		time=stuck_time;
	}
	
	return flag;
}

void Shoot_Reload(Shoot_t* shoot)
{
	shoot->dial.dial_work_time=0;
	switch (shoot->dial.dial_work_state)
	{
		case DIAL_SLEEP:     //&&shoot->fric_ok_flag==1
			shoot->dial.dial_work_time=0;
			shoot->dial.dial_speed_target=0;
			if(shoot->shoot_load_state==LOAD_NO&&(shoot->fire_flag==1||shoot->firing_flag==1)&&shoot->block_flag==0)
			{
				shoot->dial.dial_work_state=DIAL_RELOAD;
				
			}
		
			break;
		
		case DIAL_RELOAD:
			switch (shoot->dial.dial_mode)
			{
				case DIAL_SPEED:
					if(shoot->firing_flag==1)
					{
						shoot->dial.dial_speed_target=DIAL_RELOAD_SPEED;
				    shoot->dial.dial_angle_sum=shoot->dial.dial_config->rx_info->encoder_sum;
			  	  if(Motor_Stuck_Check(shoot->dial.dial_config,50,12000,250)==1)
			      {
				      shoot->dial.dial_work_state=DIAL_RECOIL;
				      shoot->block_flag=1;
				      shoot->dial.dial_work_time=0;
						  shoot->firing_flag=0;
			      }
			      else
			      {
				      if(shoot->dial.dial_work_time>DIAL_WORK_TIME_MAX)   //shoot->fric_ok_flag==0||
			        {
				      	shoot->dial.dial_speed_target=0;
		  	        shoot->dial.dial_work_state=DIAL_SLEEP;
				        shoot->dial.dial_work_time=0;
				      	shoot->shoot_load_state=LOAD_OK;
						  	
			        }  
				      else
			        {
				        shoot->dial.dial_work_time++;
			        }
			      }
					}
					else if(shoot->firing_flag==0)
					{
						shoot->dial.dial_speed_target=0;
						shoot->shoot_load_state=LOAD_OK;
						shoot->dial.dial_work_time=0;
					}
					
			  	break;
				
				case DIAL_ANGLE:
					shoot->dial.dial_angle_sum+=ONESHOT_ANGLE;
				
			    if(Motor_Stuck_Check(shoot->dial.dial_config,50,12000,250)==1)
			    {
				    shoot->dial.dial_work_state=DIAL_RECOIL;
				    shoot->block_flag=1;
				    shoot->dial.dial_work_time=0;
						shoot->fire_flag=0;
			    }
			    else
			    {
				    if(shoot->fric_ok_flag==0||shoot->dial.dial_work_time>DIAL_WORK_TIME_MAX)
			      {
				    	shoot->dial.dial_speed_target=0;
		  	      shoot->dial.dial_work_state=DIAL_SLEEP;
				      shoot->dial.dial_work_time=0;
				    	shoot->shoot_load_state=LOAD_OK;
							shoot->fire_flag=0;
			      }
						else if(shoot->dial.dial_config->rx_info->encoder_speed==0)
						{
						  shoot->fire_flag=0;
						  shoot->dial.dial_work_time=0;
					  }
				    else
			      {
				      shoot->dial.dial_work_time++;
			      }
			    }
					
				  break;
		  }
			
			break;
		
		case DIAL_RECOIL:
			shoot->dial.dial_angle_sum-=ONESHOT_ANGLE;
		  shoot->dial.dial_mode=DIAL_ANGLE;
			if(shoot->dial.dial_config->rx_info->encoder_sum<=shoot->dial.dial_angle_sum)
			{
				shoot->dial.dial_work_time=0;
				shoot->dial.dial_speed_target=0;
				shoot->shoot_load_state=LOAD_OK;
				shoot->block_flag=0;
				shoot->dial.dial_work_state=DIAL_SLEEP;
				
			}
			else if(shoot->dial.dial_work_time>DIAL_WORK_TIME_MAX)
			{
				shoot->dial.dial_work_time=0;
				shoot->dial.dial_speed_target=0;
				shoot->shoot_load_state=LOAD_OK;
				shoot->block_flag=0;
				shoot->dial.dial_work_state=DIAL_SLEEP;
			}
			else
			{
				shoot->dial.dial_work_time++;
			}
				
			break;
		default:
		break;
	}
}

void Shoot_PID_Calculate(Shoot_t *shoot)
{
	uint8_t i;
	
	if(shoot->dial.dial_mode==DIAL_SPEED)
	{
		shoot->dial.dial_config->ctrl->speed_ctrl->target=shoot->dial.dial_speed_target;
	  shoot->dial.dial_config->ctrl->speed_ctrl->measure=shoot->dial.dial_config->rx_info->encoder_speed;
	  shoot->dial.dial_config->ctrl->speed_ctrl->err=shoot->dial.dial_config->ctrl->speed_ctrl->target-shoot->dial.dial_config->ctrl->speed_ctrl->measure;
		
		single_pid_ctrl(shoot->dial.dial_config->ctrl->speed_ctrl);
		shoot->dial.dial_config->tx_info->torque=shoot->dial.dial_config->ctrl->speed_ctrl->out;
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
	shoot->dial.dial_config->single_set_torque(shoot->dial.dial_config);
	
	for(i=0;i<6;i++)
	{
		float k=-1;
		{
			if(i==2||i==5)
			{
				k=1;
			}
		}
		
		if(i<3)
		{
			shoot->fric.first_fric[i]->ctrl->speed_ctrl->target=k*FIRST_FRIC_SPEED_TARGET;
			shoot->fric.first_fric[i]->ctrl->speed_ctrl->measure=shoot->fric.first_fric[i]->rx_info->encoder_speed;
			shoot->fric.first_fric[i]->ctrl->speed_ctrl->err=shoot->fric.first_fric[i]->ctrl->speed_ctrl->target-shoot->fric.first_fric[i]->ctrl->speed_ctrl->measure;
		  single_pid_ctrl(shoot->fric.first_fric[i]->ctrl->speed_ctrl);
			shoot->fric.first_fric[i]->tx_info->torque=shoot->fric.first_fric[i]->ctrl->speed_ctrl->out;
		}
		else if(i>=3&&i<6)
		{
			shoot->fric.second_fric[i-3]->ctrl->speed_ctrl->target=k*SECOND_FRIC_SPEED_TARGET;
			shoot->fric.second_fric[i-3]->ctrl->speed_ctrl->measure=shoot->fric.second_fric[i-3]->rx_info->encoder_speed;
			shoot->fric.second_fric[i-3]->ctrl->speed_ctrl->err=shoot->fric.second_fric[i-3]->ctrl->speed_ctrl->target-shoot->fric.second_fric[i-3]->ctrl->speed_ctrl->measure;
		  single_pid_ctrl(shoot->fric.second_fric[i-3]->ctrl->speed_ctrl);
			shoot->fric.second_fric[i-3]->tx_info->torque=shoot->fric.second_fric[i-3]->ctrl->speed_ctrl->out;
		}
	}
}

void Fric_State_Check(Shoot_t *shoot)
{
	
	for(uint8_t i=0;i<6;i++)
	{
		if(i<3)
		{
			shoot->fric.fric_speed_fact[i]=shoot->fric.first_fric[i]->rx_info->encoder_speed;
			shoot->fric.fric_speed_err[i]=shoot->fric.fric_speed_fact[i]-FIRST_FRIC_SPEED_TARGET;
			shoot->fric.fric_temperature_fact[i]=shoot->fric.first_fric[i]->rx_info->temperature;
		}
		else if(i>=3&&i<6)
		{
			shoot->fric.fric_speed_fact[i]=shoot->fric.second_fric[i-3]->rx_info->encoder_speed;
			shoot->fric.fric_speed_err[i]=shoot->fric.fric_speed_fact[i]-SECOND_FRIC_SPEED_TARGET;
			shoot->fric.fric_temperature_fact[i]=shoot->fric.second_fric[i-3]->rx_info->temperature;
		}
	}
	
	if(shoot->fric.fric_speed_err[M_Fric_B_UP]<100&&shoot->fric.fric_speed_err[M_Fric_B_R]<100&&shoot->fric.fric_speed_err[M_Fric_B_L]<100)
	{
		shoot->fric_ok_flag=1;
	}
	if(shoot->fric.fric_speed_fact[M_Fric_B_UP]<1000||shoot->fric.fric_speed_fact[M_Fric_B_R]<1000||
		 shoot->fric.fric_speed_fact[M_Fric_B_L]<1000||shoot->fric.fric_speed_fact[M_Fric_F_UP]<1000||
		 shoot->fric.fric_speed_fact[M_Fric_F_R]<1000||shoot->fric.fric_speed_fact[M_Fric_F_L]<1000)
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

void Shoot_Sleep(Shoot_t *shoot)
{
	float t=RM_Group2.motor[1]->tx_info->torque;
	
	shoot->dial.dial_config->single_sleep(shoot->dial.dial_config);
	RM_Group1.group_sleep(&RM_Group1);
	RM_Group2.group_sleep(&RM_Group2);
	
	if(heart_cnt<70)
	{
		RM_Group2.motor[1]->tx_info->torque=t;
	}

}

void Shoot_Work(Shoot_t *shoot)
{
	Shoot_Safe_State_Update(shoot);
	switch (shoot->shoot_safe_state){
		case locked:
			Shoot_Sleep(shoot);
			
		  break;
		 
		case unlock:
			Shoot_Work_State_Update(shoot);
		  Shoot_Reload(shoot);
		  Shoot_PID_Calculate(shoot);
	    Fric_State_Check(shoot);
		  break;
		
		default:
			break;
	}
	
}






