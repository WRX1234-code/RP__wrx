#include "Gimbal.h"
#include "gimbal_motor.h"
#include "rc_sensor.h"
#include "Balance.h"
#include "Board_protocol.h"


Gimbal_t gimbal = {
	.yaw = &Yaw_Motor,
	.info = {
		.cfg_info = {
			.rc_yaw_mec_k = 0,
		  .rc_yaw_gyro_k = 0,
		  .rc_pitch_mec_k = 0,
			.rc_pitch_gyro_k = 0,
		  .key_yaw_mec_k = 0,
		  .key_yaw_gyro_k = 0,
			.key_pitch_mec_k = 0,
			.key_pitch_gyro_k = 0,
		
		},
	},
	.cmd = {
	  .pitch_mec_tar = P_ZERO_ANGLE,
		.yaw_mec_tar = Y_ZERO_ANGLE,
	
	}
};


void Gimbal_Board_Update(Gimbal_t* gimbal)
{
	gimbal->info.rt_info.pitch_imu = D_Board_Rx_Info.pitch_imu;
	gimbal->info.rt_info.pitch_mec = D_Board_Rx_Info.pitch_mec; 
	gimbal->info.rt_info.pitch_v = D_Board_Rx_Info.pitch_v;
	gimbal->info.rt_info.yaw_imu = D_Board_Rx_Info.yaw_imu;
	gimbal->info.rt_info.yaw_v = D_Board_Rx_Info.yaw_v;
	gimbal->info.rt_info.vision_pitch_tar = D_Board_Rx_Info.vision_pitch_tar;
	gimbal->info.rt_info.vision_yaw_tar = D_Board_Rx_Info.vision_yaw_tar;
	
	D_Board_Tx_Pkt.pitch_mec_tar = gimbal->cmd.pitch_mec_tar;
	D_Board_Tx_Pkt.pitch_imu_tar = gimbal->cmd.pitch_imu_tar;
	D_Board_Tx_Pkt.yaw_imu_tar = gimbal->cmd.yaw_imu_tar;
	
	
	if(D_Board_Rx_Info.vision_state == 0)
	{
		D_Board_Tx_Pkt.yaw_offset = 0;
	}
	
	if(Balance.Flag->Lob_Flag == true || Balance.Flag->Mec_Flag == true)
	{
		D_Board_Tx_Pkt.Gimbal_mode = 3;
	}
	else if(Balance.Flag->Lob_Flag == false || Balance.Flag->Mec_Flag == true)
	{
		D_Board_Tx_Pkt.Gimbal_mode = 2;
	}

	if(Balance.Flag->Turn_Flag == true || Balance.Flag->S_Turn_Flag == true)
	{
		D_Board_Tx_Pkt.Gimbal_mode = 1;
	}
	
}


void Gimbal_Reset_Init(Gimbal_t* gimbal)
{
	if(Balance.mode != Init_Mode)
  {
		return;
	}		
	gimbal->cmd.yaw_mec_tar = Y_ZERO_ANGLE;
	D_Board_Tx_Pkt.pitch_mec_tar = P_ZERO_ANGLE;
	
	gimbal->cmd.yaw_imu_tar = gimbal->info.rt_info.yaw_imu;
	gimbal->cmd.pitch_imu_tar = gimbal->info.rt_info.pitch_imu;
	
	if(fabs(Y_ZERO_ANGLE - gimbal->yaw->rx_info->motor_angle) <= 0.02f && fabs(P_ZERO_ANGLE - D_Board_Rx_Info.pitch_mec) <= 0.02f)
	{
		Balance.Flag->Gimbal_Reset_OK = true;
	}
	
	return;
}


void Gimbal_Mec_Update(Gimbal_t* gimbal)
{
	if(Balance.Flag->Mec_Flag == false)
	{
		return;	
	}
	
	Gimbal_Reset_Init(gimbal);
	
	gimbal->cmd.yaw_mec_tar = Y_ZERO_ANGLE;
  if(Balance.ctrl == RC_CTRL)
	{
	  gimbal->cmd.pitch_mec_tar += gimbal->info.cfg_info.rc_pitch_mec_k;  //需要修改
	}
	else if(Balance.ctrl == KEY_CTRL)
	{
		gimbal->cmd.pitch_mec_tar += gimbal->info.cfg_info.key_pitch_mec_k;  //需要修改
	}
		
	gimbal->cmd.pitch_mec_tar = constrain(gimbal->cmd.pitch_mec_tar,P_MEC_ANGLE_MIN, P_MEC_ANGLE_MAX);
	gimbal->cmd.yaw_imu_tar = gimbal->info.rt_info.yaw_imu;
	gimbal->cmd.pitch_imu_tar = gimbal->info.rt_info.pitch_imu;
}


void Gimbal_Gyro_Update(Gimbal_t* gimbal)
{
	#ifndef VISION_TEST
	if(Balance.Flag->Turn_Flag == false && Balance.mode != Imu_Mode) 
	{
	  return;
	}
	#else 
	if(Balance.Flag->Turn_Flag == false && Balance.mode != Imu_Mode && Balance.mode != Test_mode) 
	{
	  return;
	}
	
	#endif
	
	if(Balance.ctrl == RC_CTRL)
	{
		gimbal->cmd.yaw_imu_tar += gimbal->info.cfg_info.rc_yaw_gyro_k;
		gimbal->cmd.pitch_imu_tar += gimbal->info.cfg_info.rc_pitch_gyro_k;  //需要修改
	}
	else if(Balance.ctrl == KEY_CTRL)
	{
		gimbal->cmd.yaw_imu_tar += gimbal->info.cfg_info.key_yaw_gyro_k;
	  gimbal->cmd.pitch_imu_tar += gimbal->info.cfg_info.key_pitch_gyro_k;  //需要修改
	}
		
	gimbal->cmd.yaw_imu_tar = half_cycle(gimbal->cmd.yaw_imu_tar, 360.f);
	gimbal->cmd.pitch_imu_tar = constrain(gimbal->cmd.pitch_imu_tar,P_GYRO_ANGLE_MIN, P_GYRO_ANGLE_MAX);
		
	gimbal->cmd.pitch_mec_tar = gimbal->info.rt_info.pitch_mec;
	gimbal->cmd.yaw_mec_tar = gimbal->yaw->rx_info->motor_angle;
	
}

void Vision_Self_Aim_Update(Gimbal_t* gimbal)
{
	if(D_Board_Rx_Info.vision_state == 1)
	{
		D_Board_Tx_Pkt.yaw_offset = gimbal->yaw->rx_info->motor_angle - Y_ZERO_ANGLE;
	
		gimbal->cmd.yaw_imu_tar = D_Board_Rx_Info.vision_yaw_tar;
		gimbal->cmd.pitch_imu_tar = D_Board_Rx_Info.vision_pitch_tar;
	}
}

void Gimbal_Pid_Cal(Gimbal_t* gimbal)
{
	if(D_Board_Tx_Pkt.Gimbal_state == 0)
	{
		return;
	}
	
	if(Balance.Flag->Mec_Flag == true)
	{
		gimbal->yaw->pid->mec_pid.angle.target = gimbal->cmd.yaw_mec_tar;
		gimbal->yaw->pid->mec_pid.angle.measure = gimbal->yaw->rx_info->motor_angle;
		gimbal->yaw->pid->mec_pid.angle.err = gimbal->yaw->pid->mec_pid.angle.target - gimbal->yaw->pid->mec_pid.angle.measure;
		gimbal->yaw->pid->mec_pid.angle.err = half_cycle(gimbal->yaw->pid->mec_pid.angle.err, y_encoder_val_max - y_encoder_val_min);
		
		single_pid_ctrl(&gimbal->yaw->pid->mec_pid.angle);
		
		gimbal->yaw->pid->mec_pid.speed.target = gimbal->yaw->pid->mec_pid.angle.out;
		gimbal->yaw->pid->mec_pid.speed.measure = gimbal->info.rt_info.yaw_v;
		gimbal->yaw->pid->mec_pid.speed.err = gimbal->yaw->pid->mec_pid.speed.target - gimbal->yaw->pid->mec_pid.speed.measure;
		
		single_pid_ctrl(&gimbal->yaw->pid->mec_pid.speed);
		
		gimbal->yaw->tx_info->torque = gimbal->yaw->pid->mec_pid.speed.out;
	}         
  
  else if(Balance.mode == Imu_Mode || Balance.Flag->Turn_Flag == true || Balance.Flag->S_Turn_Flag == true)	
	{
		gimbal->yaw->pid->gyro_pid.angle.target = gimbal->cmd.yaw_imu_tar;
		gimbal->yaw->pid->gyro_pid.angle.measure = gimbal->info.rt_info.yaw_imu;
		gimbal->yaw->pid->gyro_pid.angle.err = gimbal->yaw->pid->gyro_pid.angle.target - gimbal->yaw->pid->gyro_pid.angle.measure;
		gimbal->yaw->pid->gyro_pid.angle.err = half_cycle(gimbal->yaw->pid->gyro_pid.angle.err, 360.f);
		
		single_pid_ctrl(&gimbal->yaw->pid->gyro_pid.angle);
		
		gimbal->yaw->pid->gyro_pid.speed.target = gimbal->yaw->pid->gyro_pid.angle.out;
		gimbal->yaw->pid->gyro_pid.speed.measure = gimbal->info.rt_info.yaw_v;
		gimbal->yaw->pid->gyro_pid.speed.err = gimbal->yaw->pid->gyro_pid.speed.target - gimbal->yaw->pid->gyro_pid.speed.measure;
		
		single_pid_ctrl(&gimbal->yaw->pid->gyro_pid.speed);
		
		gimbal->yaw->tx_info->torque = gimbal->yaw->pid->gyro_pid.speed.out;
	}

}

void Gimbal_Send(Gimbal_t* gimbal)
{
	if(D_Board_Tx_Pkt.Gimbal_state == 0)
	{
		gimbal->yaw->single_sleep(gimbal->yaw);
		gimbal->yaw->single_set_torque(gimbal->yaw);
	}
	else
	{
		gimbal->yaw->single_set_torque(gimbal->yaw);
	}
}

void Gimbal_Work(Gimbal_t* gimbal)
{
	Gimbal_Board_Update(gimbal);
	Gimbal_Mec_Update(gimbal);
	Gimbal_Gyro_Update(gimbal);
	Vision_Self_Aim_Update(gimbal);
	Gimbal_Pid_Cal(gimbal);
	Gimbal_Send(gimbal);

}

