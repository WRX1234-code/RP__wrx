#include "infantry.h"
#include "board_protocol.h"
#include "chassis.h"
#include "gimbal.h"
#include "launch.h" 
#include "vision.h"
#include "ui.h"
#include "cap.h"
#include "judge.h"

static void Infantry_Init(Infantry_t* infantry);
static void Rc_Status_Update(Infantry_t* infantry);
static void Key_Status_Update(Infantry_t* infantry);
static void Infantry_Flag_Clean(Infantry_t* infantry);
static void Infantry_Flag_Update(Infantry_t* infantry);
Signal_Form_e Spec_Flag_Update(Flag_Class_t* flag,uint8_t heartbeat,bool is_cnt);
static void Infantry_Status_Update(Infantry_t* infantry);
static void Infantry_Work(Infantry_t* infantry);

Infantry_t  infantry = {
	.ctrl = RC_CTRL,
	.mode = I_SLEEP,
	.flag = {
		.mec_flag = true,
	  .imu_flag = false,
    .turn_flag = false,
	  .hole_flag = false,
	  .vision_flag = 0,
	  .broken_flag = false,
		
		.chassis_off = false,
		.gimbal_off = false,
			
		.U_turn_flag= {
			.value = false,
			.tick_max = 800,	
		},
		
	  .L_turn_flag = {
			.value = false,
			.tick_max = 800,			
		},
		
	  .R_turn_flag = {
			.value = false,
			.tick_max = 800,	
		},
			
		.chassis_reset = {
			.value = false,
			.tick_max = 800,	
		
		},
		.car_reset = false,	
	
	},
	
	.init = Infantry_Init,


};


static void Infantry_Init(Infantry_t* infantry)
{
	infantry->work = Infantry_Work;
}

static uint8_t last_thumbwheel_step[4];

static void Infantry_Work(Infantry_t* infantry)
{
	Infantry_Status_Update(infantry);
	
	chassis.work(&chassis);
	gimbal.work(&gimbal);
	launch.work(&launch);
	vision.work(&vision);
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
						
					}
					else{
						infantry->mode = I_IMU;
					
					}
				}
				
				else if(WHEEL_DOWN_TO_ONCE)
				{
					infantry->flag.mec_flag = !infantry->flag.mec_flag;
					
					if(infantry->flag.mec_flag == true)
					{
						infantry->mode = I_MEC;
						
					}
					else{
            infantry->mode = I_IMU;

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
						infantry->flag.chassis_reset.value = true;
			
					}
					else{
//						infantry->mode = I_MEC;
					
					}
				}
				
				else if(WHEEL_DOWN_TO_ONCE)
				{
					if((infantry->flag.hole_flag == false && infantry->mode != I_HOLE) || infantry->flag.chassis_reset.value == false)
					{
						if(infantry->flag.U_turn_flag.value == false)
					  {
					    infantry->flag.U_turn_flag.value = true;
				  	}
					}
			
				}
			}
		
			break;
		
		case  RC_SW_MID:
			if(WHEEL_UP_TO_ONCE)
			{
				launch.state = !launch.state;
	
			}
			
			break;
		
		case  RC_SW_DOWN:
			if(rc_info->s2.value == RC_SW_UP)
			{
				if(WHEEL_UP_TO_ONCE)
				{
					if(infantry->flag.vision_flag != 1)
					{
					  infantry->flag.vision_flag = 1;
						
					}
					else{
						infantry->flag.vision_flag = 0;
					
					}
				  
				}
				else if(WHEEL_DOWN_TO_ONCE)
				{
					if(infantry->flag.vision_flag != 4)
					{
					  infantry->flag.vision_flag = 4;
					}
					else{
						infantry->flag.vision_flag = 0;
					
					}
				}
			}
			else if(rc_info->s2.value == RC_SW_MID)
			{
				if(WHEEL_UP_TO_ONCE)
				{
					if(infantry->flag.vision_flag != 2)
					{
					  infantry->flag.vision_flag = 2;
					}
					else{
						infantry->flag.vision_flag = 0;
					
					}
				}
				else if(WHEEL_DOWN_TO_ONCE)
				{
					if(infantry->flag.vision_flag != 3)
					{
					  infantry->flag.vision_flag = 3;
					}
					else{
						infantry->flag.vision_flag = 0;
					
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
	
	
	if(launch.state == L_LOCK)
	{
		launch.shoot_lock = 1;
	}
	else{
	  if(launch.shoot_lock == 0)
	  {
		  if(rc_info->s1.status == mid_R || rc_info->s1.status == up_R || rc_info->s1.status == down_R)
		  {
		    launch.shoot_lock = 1;
		  }
	  }
	  if(launch.shoot_lock == 1)
	  {
		  if(rc_info->s1.value == RC_SW_MID && rc_info->s2.value == RC_SW_MID)
		  {
			  launch.shoot_lock = 0;
		  }
	  }
				
  }
	
	if(rc_info->s1.value == RC_SW_MID)
	{
		if(rc_info->s2.value == RC_SW_UP)
    {
		  launch.mode = SINGLE_SHOT;
		  launch.shoot_level = !launch.shoot_lock;
			
			shoot_statistics.shoot_mode = 0;
		  shoot_statistics.shooting_flag=0;
		  if(launch.state == L_UNLOCK && launch.shoot_lock == 0)
		  {
			  if(rc_info->s2.status == up_R)
			  {
				  Shooting_Cmd_Excute_Tick_Calculating(0);
			  }
	  	}
	  }
	  else if(rc_info->s2.value == RC_SW_MID)
	  {
		  launch.mode = SINGLE_SHOT;
		  launch.shoot_level = 0;
			
			shoot_statistics.shoot_mode = 0;
	    shoot_statistics.shooting_flag = 0;
	  }
	  else if(rc_info->s2.value == RC_SW_DOWN)
	  {
		  launch.mode = REPEAT_SHOT;
		  launch.shoot_level = !launch.shoot_lock;
			
			if(launch.state == L_UNLOCK && launch.shoot_lock == 0)
		  {
			  shoot_statistics.shoot_mode = 1;
			  if(shoot_statistics.shooting_flag == 0 && rc_info->s2.status == keep_R)
			  {
				  Shooting_Cmd_Excute_Tick_Calculating(0);
			    shoot_statistics.shooting_flag = 1;
		  	}
		
		  }
	  }
	}
	else{
	  launch.mode = SINGLE_SHOT;
		 launch.shoot_level = 0;
		
		shoot_statistics.shoot_mode = 0;
	  shoot_statistics.shooting_flag = 0;
	}
	
	
	last_thumbwheel_step[0] = rc_info->thumbwheel.step[0];
  last_thumbwheel_step[1] = rc_info->thumbwheel.step[1];
	last_thumbwheel_step[2] = rc_info->thumbwheel.step[2];
	last_thumbwheel_step[3] = rc_info->thumbwheel.step[3];
	
}

static void Key_Status_Update(Infantry_t* infantry)
{
  rc_sensor_info_t*  rc_info = rc_sensor.info;
	
	if(rc_info->s1.value == RC_SW_UP && rc_info->s2.value == RC_SW_DOWN)
	{
	  infantry->ctrl = KEY_CTRL;
		
	}
	else{
	  infantry->ctrl = RC_CTRL;
	}
	
	if(rc_info->Shift.status == release_to_press)
	{
		infantry->mode = I_TURN;
	}
	if(rc_info->V.status == release_to_press)
	{
		infantry->flag.hole_flag = true;
		infantry->flag.chassis_reset.value = true;
	  infantry->mode = I_HOLE;
	
	}
	
	
	if((infantry->flag.hole_flag == false && infantry->mode != I_HOLE) || infantry->flag.chassis_reset.value == false)
	{
	  if(infantry->flag.R_turn_flag.value == false && infantry->flag.L_turn_flag.value == false)
	  {
      if(rc_info->R.status == release_to_press)
	    {
		    if(infantry->flag.U_turn_flag.value == false)
		    {
			    infantry->flag.U_turn_flag.value = true;
		    }
	    }
	  }
	
	  if(infantry->flag.U_turn_flag.value == false && infantry->flag.L_turn_flag.value == false)
	  {
      if(rc_info->E.status == release_to_press)
	    {
		    if(infantry->flag.R_turn_flag.value == false)
		    {
			    infantry->flag.R_turn_flag.value = true;
		    }
	    }
	  }
		
	  if(infantry->flag.U_turn_flag.value == false && infantry->flag.R_turn_flag.value == false)
	  {
      if(rc_info->Q.status == release_to_press)
	    {
		    if(infantry->flag.L_turn_flag.value == false)
		    {
			    infantry->flag.L_turn_flag.value = true;
		    }
	    }
	  }
	
	}
	
	
	if(rc_info->Z.status == release_to_press)
	{
		infantry->flag.vision_flag = 2;
	}
	else if(rc_info->X.status == release_to_press)
	{
		infantry->flag.vision_flag = 3;
	}
	else if(rc_info->C.status == release_to_press)
	{
		infantry->flag.vision_flag = 4;
	}
	
	if(infantry->flag.vision_flag <= 1)
	{
		if(rc_info->mouse_btn_r.status == long_press)
		{
			infantry->flag.vision_flag = 1;
		}
	}
	
	if(rc_info->mouse_btn_l.cnt == 0)
	{
		launch.shoot_level = 0;
	}
	else{
	  launch.shoot_level = 1;
	}
	
	
	if(rc_info->B.status == release_to_press)
	{
		launch.state = !launch.state;
	}
	
	
	if(rc_info->Ctrl.status == release_to_press)
	{
		if(infantry->mode == I_HOLE)
		{
			infantry->flag.hole_flag = false;
		}
		else{
		  infantry->mode = I_IMU;
		
		  infantry->flag.chassis_reset.value = true;
//	    infantry->flag.car_reast = true;
		}
		
	}
	
}


static void Infantry_Flag_Clean(Infantry_t* infantry)
{
  infantry->flag.mec_flag = true;
	infantry->flag.imu_flag = false;
  infantry->flag.turn_flag = false;
	infantry->flag.hole_flag = false;
	infantry->flag.vision_flag = 0;
	infantry->flag.broken_flag = false;
	
  infantry->flag.U_turn_flag.value = false;
	infantry->flag.L_turn_flag.value = false;
	infantry->flag.R_turn_flag.value = false;

	infantry->flag.chassis_reset.value = false;
	infantry->flag.car_reset = false;

}


static void Infantry_Flag_Update(Infantry_t* infantry)
{
	if(infantry->mode == I_SLEEP || infantry->mode == I_INIT || infantry->mode == I_MEC)
	{
		infantry->flag.mec_flag = true;
		infantry->flag.imu_flag = false;
		infantry->flag.turn_flag = false;
		infantry->flag.hole_flag = false;
		
	}
	else if(infantry->mode == I_IMU)
	{
		infantry->flag.imu_flag = true;
		infantry->flag.mec_flag = false;
		infantry->flag.turn_flag = false;
		infantry->flag.hole_flag = false;
		
	}
	else if(infantry->mode == I_TURN)
	{
		infantry->flag.turn_flag = true;
		infantry->flag.mec_flag = false;
		infantry->flag.imu_flag = false;
		infantry->flag.hole_flag = false;
		
	}
	else if(infantry->mode == I_HOLE)
	{
//		infantry->flag.hole_flag = true;
		infantry->flag.mec_flag = false;
		infantry->flag.imu_flag = false;
		infantry->flag.turn_flag = false;
		
	}
	
	if(infantry->flag.hole_flag == true)
	{
		infantry->flag.vision_flag = 0;
	}
  if(infantry->flag.broken_flag == true)
	{
		if(infantry->mode != I_SLEEP && infantry->mode != I_INIT)
		{
			infantry->mode = I_MEC;
		}
		
		infantry->flag.mec_flag = true;
		infantry->flag.imu_flag = false;
		infantry->flag.turn_flag = false;
	  infantry->flag.hole_flag = false;
	}
	
	if(infantry->flag.vision_flag != 0)
	{
		if(infantry->flag.mec_flag == true)
		{
			infantry->flag.mec_flag = false;
			infantry->flag.imu_flag = true;
			
			infantry->mode = I_IMU;
			
		}
	}
	
	Spec_Flag_Update(&infantry->flag.U_turn_flag,(infantry->mode > I_INIT),true);
	Spec_Flag_Update(&infantry->flag.R_turn_flag,(infantry->mode > I_INIT),true);
	Spec_Flag_Update(&infantry->flag.L_turn_flag,(infantry->mode > I_INIT),true);
	Spec_Flag_Update(&infantry->flag.chassis_reset,(infantry->mode > I_INIT),true);
	
}


Signal_Form_e Spec_Flag_Update(Flag_Class_t* flag,uint8_t heartbeat,bool is_cnt)
{
	if(heartbeat == 0)
	{
		flag->tick = 0;
			
		flag->value = false;
	}
	else if(flag->value == true)
	{
		if(is_cnt == true)
		{
			flag->tick ++;
		
		  if(flag->tick >= flag->tick_max)
		  {
			  flag->tick = 0;
			
			  flag->value = false;
		  }
		}
		
	}
	else{
		flag->tick = 0;
	}
	
	if(flag->value == false && flag->last_value == false)
	{
		flag->form = LOWING;
	}
	else if(flag->value == true && flag->last_value == false)
	{
		flag->form = RISING;
	}
	else if(flag->value == true && flag->last_value == true)
	{
		flag->form = HIGHING;
	}
	else if(flag->value == false && flag->last_value == true)
	{
		flag->form = FALLING;
	}
	
	flag->last_value = flag->value;
	
	return flag->form;
}


static void Infantry_Status_Update(Infantry_t* infantry)
{
	static bool last_c_off = false;
	static bool last_g_off = false;
	
	rc_sensor_info_t*  rc_info = rc_sensor.info;
	if(rc_sensor.work_state == DEV_OFFLINE || (infantry->flag.chassis_off == true && infantry->flag.gimbal_off == true))
	{
		infantry->mode = I_SLEEP;
		
		launch.state = L_LOCK;
		launch.shoot_lock = 1;
		infantry->flag.vision_flag = 0;
		
		Infantry_Flag_Clean(infantry);
		
		last_thumbwheel_step[0] = rc_info->thumbwheel.step[0];
		last_thumbwheel_step[1] = rc_info->thumbwheel.step[1];
		last_thumbwheel_step[2] = rc_info->thumbwheel.step[2];
		last_thumbwheel_step[3] = rc_info->thumbwheel.step[3];
		
	}
	
	else{
	
	  if(infantry->mode == I_SLEEP || (infantry->flag.chassis_off == false && last_c_off == true))
	  {  
		  infantry->mode = I_INIT;
		 
		  launch.state = L_LOCK;
			launch.shoot_lock = 1;
		  infantry->flag.vision_flag = 0;
			
			Infantry_Flag_Clean(infantry);
		  cap_tx_info.bit_control.pre_charge_mode_en = 0;
	  }
	  else if(infantry->mode == I_INIT)
	  {
		  launch.state = L_LOCK;
			launch.shoot_lock = 1;
		  infantry->flag.vision_flag = 0;
		
			if(infantry->flag.gimbal_off == true)
	  	{
			  infantry->mode = I_MEC;
			
	  	}
			else if(infantry->flag.broken_flag == true)
		  {
			 	infantry->mode = I_MEC;
		  }
		  else if(gimbal.gimbal_reset_flag == true)
			{  
			  infantry->mode = I_IMU;
		  }
			
			 last_thumbwheel_step[0] = rc_info->thumbwheel.step[0];
			 last_thumbwheel_step[1] = rc_info->thumbwheel.step[1];
			 last_thumbwheel_step[2] = rc_info->thumbwheel.step[2];
			 last_thumbwheel_step[3] = rc_info->thumbwheel.step[3];
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
			
			
	    if(infantry->flag.gimbal_off == true && last_g_off == false)
	  	{
			  infantry->mode = I_MEC;
	  	}
  	}
	}
	
	Infantry_Flag_Update(infantry);
	
	
	last_c_off = infantry->flag.chassis_off;
	last_g_off = infantry->flag.gimbal_off;
	
}




