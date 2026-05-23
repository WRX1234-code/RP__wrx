#include "gimbal.h"
#include "infantry.h"
#include "board_protocol.h"
#include "rp_math.h"
#include "rc_sensor.h"


static void Gimbal_Data_Update(Gimbal_t* gimbal);
static void  Gimbal_Slave_Update(Gimbal_t* gimbal);
static void  Gimbal_Boss_Update(Gimbal_t* gimbal);
static void Gimbal_Offline_Process(Gimbal_t* gimbal);
static void Gimbal_Work(Gimbal_t* gimbal);


Gimbal_t gimbal = {
  .mode = G_SLEEP,
  .gimbal_reset_flag= false,
		
	.config = {
		.yaw_zero[FRONT] = YAW_MEC_ZERO_ANGLE,
	
	},
	.work = Gimbal_Work,
};

static void Gimbal_Data_Update(Gimbal_t* gimbal)
{
  gimbal->info.yaw_mec = board.rx_meg->gimbal_meg.yaw_mec;
	gimbal->info.yaw_imu = board.rx_meg->gimbal_meg.yaw_imu;
  gimbal->info.pitch_mec = board.rx_meg->gimbal_meg.pitch_mec;
	gimbal->info.pitch_imu = board.rx_meg->gimbal_meg.pitch_imu;

	gimbal->info.yaw_mec_err = motor_half_cycle(gimbal->info.yaw_mec - YAW_MEC_ZERO_ANGLE,2*PI);
	
	board.tx_pkt->gimbal_target_pkt.yaw_mec_tar = gimbal->target.yaw_mec_tar;
	board.tx_pkt->gimbal_target_pkt.yaw_imu_tar = gimbal->target.yaw_imu_tar;
	board.tx_pkt->gimbal_target_pkt.pitch_mec_tar = gimbal->target.pitch_mec_tar;
	board.tx_pkt->gimbal_target_pkt.pitch_imu_tar = gimbal->target.pitch_imu_tar;

};
	

static void  Gimbal_Slave_Update(Gimbal_t* gimbal)
{
	
	if(gimbal->gimbal_reset_flag == false || gimbal->mode == G_INIT)
	{
	  gimbal->target.yaw_mec_tar = YAW_MEC_ZERO_ANGLE;
		gimbal->target.pitch_mec_tar = PITCH_MEC_ZERO_ANGLE;
	
	}
	else{
		if(gimbal->info.yaw_mec_err <= PI/2)
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
 

}


static void  Gimbal_Boss_Update(Gimbal_t* gimbal)
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

	gimbal->target.yaw_imu_tar = motor_half_cycle(gimbal->target.yaw_imu_tar,360.f);
	gimbal->target.pitch_imu_tar = constrain(gimbal->target.pitch_imu_tar,PITCH_IMU_MIN_ANGLE,PITCH_IMU_MAX_ANGLE);
	
	gimbal->target.yaw_mec_tar = gimbal->info.yaw_mec;
	gimbal->target.pitch_mec_tar = gimbal->info.pitch_mec;
}


static void Gimbal_Offline_Process(Gimbal_t* gimbal)
{
	gimbal->gimbal_reset_flag = false;
	
	gimbal->target.pitch_mec_tar = PITCH_MEC_ZERO_ANGLE;
	gimbal->target.yaw_mec_tar = YAW_MEC_ZERO_ANGLE;


}




static void Gimbal_Work(Gimbal_t* gimbal)
{
	Gimbal_Data_Update(gimbal);
	
	switch (gimbal->mode)
	{
		case G_SLEEP:
			Gimbal_Offline_Process(gimbal);
			
			break;
		
		case G_INIT:
		case G_SLAVE:
			Gimbal_Slave_Update(gimbal);
		 break;
		
		case G_BOSS:
			Gimbal_Boss_Update(gimbal);
			break;
			
	
	  default:
			break;
	}

}