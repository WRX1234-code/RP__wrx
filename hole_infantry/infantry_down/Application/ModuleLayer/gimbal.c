#include "gimbal.h"
#include "vision.h"
#include "infantry.h"
#include "board_protocol.h"
#include "rp_math.h"
#include "rc_sensor.h"


static void Gimbal_Data_Update(Gimbal_t* gimbal);
static void Gimbal_Status_Update(Gimbal_t* gimbal);
static void Gimbal_Slave_Update(Gimbal_t* gimbal);
static void Gimbal_Boss_Update(Gimbal_t* gimbal);
static void Gimbal_Offline_Update(Gimbal_t* gimbal);
static void Gimbal_Offline_Process(Gimbal_t* gimbal);
static void Gimbal_Cmd_Transmit(Gimbal_t* gimbal);
static void Gimbal_Work(Gimbal_t* gimbal);


Gimbal_t gimbal = {
  .mode = G_SLEEP,
  .gimbal_reset_flag= false,
		
	.config = {
		.yaw_zero[FRONT] = YAW_MEC_ZERO_ANGLE,
	
	},
	.work = Gimbal_Work,
};



static void Gimbal_Status_Update(Gimbal_t* gimbal)
{
	switch (infantry.mode)
	{
	  case I_SLEEP:
			gimbal->mode = G_SLEEP;
			break;
		
		case I_INIT:
			gimbal->mode = G_INIT;
			break;
		
		case I_MEC:
			gimbal->mode = G_SLAVE;
			break;
		
		case I_HOLE:
			 if(infantry.flag.hole_flag == true && infantry.flag.chassis_reset.value == true)
			{
				gimbal->mode = G_BOSS;
			}
			else if(infantry.flag.hole_flag == true && infantry.flag.chassis_reset.value == false)
			{
				gimbal->mode = G_SLAVE;
			}
			else if(infantry.flag.hole_flag == false && board.rx_meg->gimbal_meg.is_reach == false)
			{
				gimbal->mode = G_SLAVE;
			}
			else if(infantry.flag.hole_flag == false && board.rx_meg->gimbal_meg.is_reach == true)
			{
				gimbal->mode = G_BOSS;
			}
			break;
			
		case I_IMU:
		case I_TURN:
			gimbal->mode = G_BOSS;
			break;
		
		default:
			break;
	}
	
}


static void Gimbal_Data_Update(Gimbal_t* gimbal)
{
  gimbal->info.yaw_mec = board.rx_meg->gimbal_meg.yaw_mec;
	gimbal->info.yaw_imu = board.rx_meg->gimbal_meg.yaw_imu;
  gimbal->info.pitch_mec = board.rx_meg->gimbal_meg.pitch_mec;
	gimbal->info.pitch_imu = board.rx_meg->gimbal_meg.pitch_imu;

	gimbal->info.yaw_mec_err_raw = motor_half_cycle(gimbal->info.yaw_mec - YAW_MEC_ZERO_ANGLE,2*PI);
	
	gimbal->info.pitch_mec_err_raw = motor_half_cycle(gimbal->info.pitch_mec - PITCH_MEC_ZERO_ANGLE,2*PI);
	
//	gimbal->info.yaw_mec_err_act = motor_half_cycle(gimbal->info.yaw_mec - gimbal->target.yaw_mec_tar,2*PI);
};
	
static uint16_t reset_tick = 0;
static void Gimbal_Init_Process(Gimbal_t* gimbal)
{
	gimbal->target.yaw_mec_tar = YAW_MEC_ZERO_ANGLE;
	gimbal->target.pitch_mec_tar = PITCH_MEC_ZERO_ANGLE;
	
	reset_tick ++;
	
	if(fabs(gimbal->info.yaw_mec_err_raw) <= 5.f/180.f*PI && fabs(gimbal->info.pitch_mec_err_raw) <= 5.f/180.f*PI)
	{
		gimbal->gimbal_reset_flag = true;
		reset_tick = 0;
	}
	else if(reset_tick >= 4000)
	{
		gimbal->gimbal_reset_flag = true;
		reset_tick = 0;
	}
}


static void Gimbal_Direct_Update(Gimbal_t* gimbal)
{
	if(fabs(gimbal->info.yaw_mec_err_raw) <= PI/4)
	{
		
	}
	else if(fabs(gimbal->info.yaw_mec_err_raw) >= 3*PI/4)
	{
		
	}
	else if(gimbal->info.yaw_mec_err_raw > PI/4 && gimbal->info.yaw_mec_err_raw < 3*PI/4)
	{
		
	}
	else if(gimbal->info.yaw_mec_err_raw < -PI/4 && gimbal->info.yaw_mec_err_raw > -3*PI/4)
	{
		
	}
}



static void  Gimbal_Slave_Update(Gimbal_t* gimbal)
{
	
	if(gimbal->gimbal_reset_flag == false || gimbal->mode == G_INIT)
	{
	  gimbal->target.yaw_mec_tar = YAW_MEC_ZERO_ANGLE;
		gimbal->target.pitch_mec_tar = PITCH_MEC_ZERO_ANGLE;
	
	}
	else{
		if(gimbal->info.yaw_mec_err_raw <= PI/2)
		{
			gimbal->target.yaw_mec_tar = gimbal->config.yaw_zero[FRONT];
		}
		else{
		  gimbal->target.yaw_mec_tar = gimbal->config.yaw_zero[BEHIND];
		
		}
		
		if(infantry.ctrl == RC_CTRL)
		{
			gimbal->target.pitch_mec_tar += rc_sensor.info->ch1/660.f * gimbal->config.rc_pitch_mec_step;
		}
		else if(infantry.ctrl == KEY_CTRL)
		{
			gimbal->target.pitch_mec_tar += rc_sensor.info->mouse_y * gimbal->config.key_pitch_mec_step;
		}
		
		gimbal->target.pitch_mec_tar = motor_half_cycle(gimbal->target.pitch_mec_tar,2*PI);
		gimbal->target.pitch_mec_tar = constrain(gimbal->target.pitch_mec_tar,PITCH_MEC_MIN_ANGLE,PITCH_MEC_MAX_ANGLE);
	
	  gimbal->target.yaw_imu_tar = gimbal->info.yaw_imu;
		gimbal->target.pitch_imu_tar = gimbal->info.pitch_imu;
	}
 
//	gimbal->info.yaw_mec_err_act = motor_half_cycle(gimbal->info.yaw_mec - gimbal->target.yaw_mec_tar,2*PI);

}


static void  Gimbal_Boss_Update(Gimbal_t* gimbal)
{
	if(vision.mode != V_NORMAL && board.rx_meg->state_meg.vision_state == true && board.rx_meg->vision_meg.is_find_target == true)
	{
		gimbal->target.yaw_imu_tar = board.rx_meg->vision_meg.vision_yaw_tar;
	  gimbal->target.pitch_imu_tar = board.rx_meg->vision_meg.vision_pitch_tar;
	}

	else
	{
	  if(infantry.ctrl == RC_CTRL)
    {
		  gimbal->target.yaw_imu_tar += rc_sensor.info->ch0/660.f * gimbal->config.rc_yaw_imu_step;
		  gimbal->target.pitch_imu_tar += rc_sensor.info->ch1/660.f * gimbal->config.rc_pitch_imu_step;
			
	  }
	  else if(infantry.ctrl == KEY_CTRL)
	  {
		  gimbal->target.yaw_imu_tar += rc_sensor.info->mouse_x * gimbal->config.key_yaw_imu_step;
		  gimbal->target.pitch_imu_tar += rc_sensor.info->mouse_y * gimbal->config.key_pitch_imu_step;
			
	  }

	}
	
	gimbal->target.yaw_imu_tar = motor_half_cycle(gimbal->target.yaw_imu_tar,360.f);
	gimbal->target.pitch_imu_tar = constrain(gimbal->target.pitch_imu_tar,PITCH_IMU_MIN_ANGLE,PITCH_IMU_MAX_ANGLE);
	
	if(infantry.flag.chassis_reset.value == true)
	{
		gimbal->target.yaw_mec_tar = gimbal->config.yaw_zero[FRONT];
	}
	else{
		if(fabs(gimbal->info.yaw_mec_err_raw) <= PI/2)
		{
			gimbal->target.yaw_mec_tar = gimbal->config.yaw_zero[FRONT];
		}
		else{
			gimbal->target.yaw_mec_tar = gimbal->config.yaw_zero[BEHIND];
		}
	}
			
//	gimbal->target.yaw_mec_tar = gimbal->info.yaw_mec;
	
	gimbal->target.pitch_mec_tar = gimbal->info.pitch_mec;
//	gimbal->info.yaw_mec_err_act = motor_half_cycle(gimbal->info.yaw_mec - gimbal->target.yaw_mec_tar,2*PI);
}



static void Gimbal_Offline_Update(Gimbal_t* gimbal)
{
	
}


static void Gimbal_Offline_Process(Gimbal_t* gimbal)
{
	gimbal->gimbal_reset_flag = false;
	reset_tick = 0;
	
	gimbal->target.pitch_mec_tar = PITCH_MEC_ZERO_ANGLE;
	gimbal->target.yaw_mec_tar = YAW_MEC_ZERO_ANGLE;

}


static void Gimbal_Cmd_Transmit(Gimbal_t* gimbal)
{
  board.tx_pkt->gimbal_target_pkt.yaw_mec_tar = gimbal->target.yaw_mec_tar;
	board.tx_pkt->gimbal_target_pkt.yaw_imu_tar = gimbal->target.yaw_imu_tar;
	board.tx_pkt->gimbal_target_pkt.pitch_mec_tar = gimbal->target.pitch_mec_tar;
	board.tx_pkt->gimbal_target_pkt.pitch_imu_tar = gimbal->target.pitch_imu_tar;
	
	if(infantry.flag.U_turn_flag.value == true || infantry.flag.R_turn_flag.value == true || infantry.flag.L_turn_flag.value == true)
	{
		gimbal->info.yaw_mec_err_act = 0;
	}
	else{
	  gimbal->info.yaw_mec_err_act = motor_half_cycle(gimbal->info.yaw_mec - gimbal->target.yaw_mec_tar,2*PI);
	}
	
	
	if(gimbal->mode == G_SLEEP || gimbal->mode == G_INIT || gimbal->mode == G_SLAVE)
	{
		board.tx_pkt->car_pkt.gimbal_mode = 0;
	}
	else{
	  board.tx_pkt->car_pkt.gimbal_mode = 1;
	}
	
	if(infantry.flag.hole_flag == false)
	{
		board.tx_pkt->gimbal_target_pkt.is_hole = false;
	}
	else if(infantry.flag.chassis_reset.value == true)
	{
		board.tx_pkt->gimbal_target_pkt.is_hole = false;
	}	
	else{
	  board.tx_pkt->gimbal_target_pkt.is_hole = true;
	}
  

}


static void Gimbal_Work(Gimbal_t* gimbal)
{
	Gimbal_Data_Update(gimbal);
	Gimbal_Offline_Update(gimbal);
	Gimbal_Status_Update(gimbal);
	switch (gimbal->mode)
	{
		case G_SLEEP:
			Gimbal_Offline_Process(gimbal);
			break;
		
		case G_INIT:
			Gimbal_Init_Process(gimbal);
		  break;

		case G_SLAVE:
			Gimbal_Slave_Update(gimbal);
		  break;
		
		case G_BOSS:
			Gimbal_Boss_Update(gimbal);
			break;
	
	  default:
			break;
	}
	
	Gimbal_Cmd_Transmit(gimbal);

}

