#include "infantry.h"
#include "board_protocol.h"
#include "chassis.h"
#include "gimbal.h"
#include "launch.h" 
#include "cap.h"

static void Infantry_Work(Infantry_t* infantry);

Infantry_t  infantry = {
	.ctrl = RC_CTRL,
	.mode = I_SLEEP,
	.vision = NO_VIS,
	.flag = {
		.mec_flag = true,
	  .imu_flag = false,
    .turn_flag = false,
	  .hole_flag = false,
	  .vision_flag = false,
	  .broken_flag = false,
		.U_turn_flag = false,
	  .L_turn_flag = false,
	  .R_turn_flag = false,
			
	 
	},
	
	.work = Infantry_Work,

};

static uint8_t last_thumbwheel_step[4];

static void Infantry_Work(Infantry_t* infantry)
{
	
}





static void Rc_Status_Update(Infantry_t* infantry)
{
	
	rc_sensor_info_t*  rc_info = rc_sensor.info;
	
	switch (rc_info->s1.value)
	{
		case  RC_SW_UP:
			
		  if(rc_info->s2.value == RC_SW_UP)
			{
				if(WHEEL_UP_TO_ONCE)
				{
					infantry->flag.turn_flag = !infantry->flag.turn_flag;
					
					if(infantry->flag.turn_flag == true)
					{
						infantry->mode = I_TURN;
						chassis.mode = C_BOSS;
						gimbal.mode = G_BOSS;
						
						infantry->flag.mec_flag = true;
	          infantry->flag.imu_flag = false;
//	          infantry->flag.hole_flag = false,
						
					}
					else{
						infantry->mode = I_IMU;
						chassis.mode = C_SLAVE;
						gimbal.mode = G_BOSS;
					
						infantry->flag.mec_flag = false;
						infantry->flag.imu_flag = true;
						infantry->flag.turn_flag = false;
					}
				}
				
				else if(WHEEL_DOWN_TO_ONCE)
				{
					infantry->flag.mec_flag = !infantry->flag.mec_flag;
					
					if(infantry->flag.mec_flag == true)
					{
						infantry->mode = I_MEC;
						chassis.mode = C_BOSS;
						gimbal.mode = G_SLAVE;
					
						infantry->flag.imu_flag = false;
						infantry->flag.turn_flag = false;
						
					}
					else{
            infantry->mode = I_IMU;
						chassis.mode = C_SLAVE;
						gimbal.mode = G_BOSS;
					
						infantry->flag.mec_flag = false;
						infantry->flag.imu_flag = true;
						infantry->flag.turn_flag = false;
					}						
				}
			}
			else if(rc_info->s2.value == RC_SW_MID)
			{
			  if(WHEEL_UP_TO_ONCE)
				{
					infantry->flag.hole_flag = !infantry->flag.hole_flag;
					
					if(infantry->flag.hole_flag == true)
					{
						infantry->mode = I_HOLE;
						chassis.mode = C_BOSS;
						gimbal.mode = G_SLAVE;
					
						infantry->flag.imu_flag = false;
						infantry->flag.turn_flag = false;
					}
					else{
						infantry->mode = I_IMU;
						chassis.mode = C_SLAVE;
						gimbal.mode = G_BOSS;
					
						infantry->flag.mec_flag = false;
						infantry->flag.imu_flag = true;
						infantry->flag.turn_flag = false;
					
					}
				}
				
				else if(WHEEL_DOWN_TO_ONCE)
				{
					if(infantry->flag.U_turn_flag == false)
					{
					  infantry->flag.U_turn_flag = true;
					}
					
				}
			}
		
			break;
		
		case  RC_SW_MID:
			if(WHEEL_UP_TO_ONCE)
			{
				launch.state = !launch.state;
	
			}
			
			if(launch.state == L_LOCK)
			{
				 launch.shoot_lock = 0;
			}
			else{
				if(launch.shoot_lock == 0)
			  {
				  if(rc_info->s1.status == mid_R)
				  {
					  launch.shoot_lock = 1;
				  }
			  }
			  if(launch.shoot_lock == 1)
			  {
				  if(rc_info->s2.value == RC_SW_MID)
				  {
				    launch.shoot_lock = 0;
				  }
			  }
				
			}
		
			if(rc_info->s2.value == RC_SW_UP)
		  {
				launch.mode = SINGLE_SHOT;
				launch.shoot_level = !launch.shoot_lock;
			}
			else if(rc_info->s2.value == RC_SW_MID)
			{
				launch.mode = SINGLE_SHOT;
				launch.shoot_level = 0;
			}
			else if(rc_info->s2.value == RC_SW_DOWN)
			{
				launch.mode = REPEAT_SHOT;
				launch.shoot_level = !launch.shoot_lock;
				
			}
			

			break;
		
		case  RC_SW_DOWN:
			if(rc_info->s2.value == RC_SW_UP)
			{
				if(WHEEL_UP_TO_ONCE)
				{
					if(infantry->vision != AUTO_AIM)
					{
					  infantry->vision = AUTO_AIM;
					}
					else{
						infantry->vision = NO_VIS;
					
					}
				  
				}
				else if(WHEEL_DOWN_TO_ONCE)
				{
					if(infantry->vision != OUTPOST)
					{
					  infantry->vision = OUTPOST;
					}
					else{
						infantry->vision = NO_VIS;
					
					}
				}
			}
			else if(rc_info->s2.value == RC_SW_MID)
			{
				if(WHEEL_UP_TO_ONCE)
				{
					if(infantry->vision != S_BUFF)
					{
					  infantry->vision = S_BUFF;
					}
					else{
						infantry->vision = NO_VIS;
					
					}
				}
				else if(WHEEL_DOWN_TO_ONCE)
				{
					if(infantry->vision != B_BUFF)
					{
					  infantry->vision = B_BUFF;
					}
					else{
						infantry->vision = NO_VIS;
					
					}
				}
			}
			else if(rc_info->s2.value == RC_SW_DOWN)
			{
				if(WHEEL_UP_TO_ONCE)
				{
					cap_tx_info.bit_control.pre_charge_mode_en = !cap_tx_info.bit_control.pre_charge_mode_en;
				}
				else if(WHEEL_DOWN_TO_ONCE)
				{
					
				}
			}
			
			break;
		
		default:
			break;
		
		
	}
	
	if(rc_info->s1.value == RC_SW_UP && rc_info->s2.value == RC_SW_DOWN)
	{
	  infantry->ctrl = KEY_CTRL;
	}
	else{
	  infantry->ctrl = RC_CTRL;
	}
	
	
	
	
	last_thumbwheel_step[0] = rc_info->thumbwheel.step[0];
  last_thumbwheel_step[1] = rc_info->thumbwheel.step[1];
	last_thumbwheel_step[2] = rc_info->thumbwheel.step[2];
	last_thumbwheel_step[3] = rc_info->thumbwheel.step[3];
	
	
	
	
	
	
	
}

static void Key_Status_Update(Infantry_t* infantry)
{

}



static void Infantry_Status_Update(Infantry_t* infantry)
{
	rc_sensor_info_t*  rc_info = rc_sensor.info;
	if(rc_sensor.work_state == DEV_OFFLINE)
	{
		infantry->mode = I_SLEEP;
		chassis.mode = C_SLEEP;
		gimbal.mode = G_SLEEP;
		launch.state = L_LOCK;
		infantry->vision = NO_VIS;
	}
	else if(infantry->mode == I_SLEEP)
	{
		infantry->mode = I_INIT;
		chassis.mode = C_INIT;
		gimbal.mode = G_INIT;
		launch.state = L_LOCK;
		infantry->vision = NO_VIS;
		cap_tx_info.bit_control.pre_charge_mode_en = 0;
	}
	else if(infantry->mode == I_INIT)
	{
		chassis.mode = C_INIT;
		gimbal.mode = G_INIT;
		launch.state = L_LOCK;
		infantry->vision = NO_VIS;
		
		if(gimbal.gimbal_reset_flag == true)
		{
			if(infantry->flag.broken_flag == true)
			{
				infantry->mode = I_MEC;
				chassis.mode = C_BOSS;
		    gimbal.mode = G_SLAVE;
		   
			}
			else{
			  infantry->mode = I_IMU;
			  chassis.mode = C_SLAVE;
		    gimbal.mode = G_BOSS;
			}
			
			last_thumbwheel_step[0] = rc_info->thumbwheel.step[0];
			last_thumbwheel_step[1] = rc_info->thumbwheel.step[1];
			last_thumbwheel_step[2] = rc_info->thumbwheel.step[2];
			last_thumbwheel_step[3] = rc_info->thumbwheel.step[3];
			
		}
	}
	else{
		if(infantry->ctrl == RC_CTRL)
		{
			Rc_Status_Update(infantry);
		}
	  else if(infantry->ctrl == KEY_CTRL)
		{
			Key_Status_Update(infantry);
		}
	
	}
	
	
}