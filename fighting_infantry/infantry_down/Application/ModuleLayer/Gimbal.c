#include "Gimbal.h"
#include "chassis.h"
#include "Filter.h"
#include "gimbal_motor.h"
#include "rc_sensor.h"
#include "Balance.h"
#include "Board_protocol.h"


Gimbal_t gimbal = {
	.yaw = &Yaw_Motor,
	.info = {
		.cfg_info = {
			.rc_yaw_mec_k = 0,
		  .rc_yaw_gyro_k = 0.3f,
		  .rc_pitch_mec_k = 0.005f,
			.rc_pitch_gyro_k = 0.15f,
		  .key_yaw_mec_k = 0,
		  .key_yaw_gyro_k = 0.002f,
			.key_pitch_mec_k = 0.00005f,
			.key_pitch_gyro_k = 0.0025f,
			
			.head_to[0] = Y_ZERO_ANGLE,
			.head_to[4] = 2.39645028f,
		
		},
	},
	.cmd = {
	  .pitch_mec_tar = P_ZERO_ANGLE,
		.yaw_mec_tar = Y_ZERO_ANGLE,
	  .pitch_imu_tar = 0,
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
	
	
	if(D_Board_Tx_Pkt.vision_mode == 0)
	{
		D_Board_Tx_Pkt.yaw_offset = 0;
	}
	
//	if(Balance.Flag->Lob_Flag == true || Balance.Flag->Mec_Flag == true)
//	{
//		D_Board_Tx_Pkt.Gimbal_mode = 0;
//	}

//	if(Balance.Flag->Turn_Flag == true || Balance.Flag->S_Turn_Flag == true || Balance.Flag->Imu_Flag == true)
//	{
//		D_Board_Tx_Pkt.Gimbal_mode = 1;
//	}
	
	gimbal->misc.pitch_included_angle = D_Board_Rx_Info.pitch_mec - P_ZERO_ANGLE;
	gimbal->misc.pitch_included_angle = half_cycle(gimbal->misc.pitch_included_angle,2*PI);
	
	gimbal->misc.yaw_included_angle = gimbal->yaw->rx_info->motor_angle - Y_ZERO_ANGLE;
	gimbal->misc.yaw_included_angle = half_cycle(gimbal->misc.yaw_included_angle,2*PI);
	
	D_Board_Tx_Pkt.v_x = arm_cos_f32(gimbal->misc.yaw_included_angle) * XEstimateKF.FilteredValue[1];
	D_Board_Tx_Pkt.v_y = arm_sin_f32(gimbal->misc.yaw_included_angle) * XEstimateKF.FilteredValue[1];
	
}


void Gimbal_Reset_Init(Gimbal_t* gimbal)
{
	if(Balance.mode != Init_Mode)
  {
		return;
	}		
	
	if(fabs(gimbal->misc.yaw_included_angle) <= PI/2)
	{
		gimbal->cmd.yaw_mec_tar = Y_ZERO_ANGLE;
	}
	else if(fabs(gimbal->misc.yaw_included_angle) > PI/2)
	{
		gimbal->cmd.yaw_mec_tar = gimbal->info.cfg_info.head_to[4];
	}
	D_Board_Tx_Pkt.pitch_mec_tar = P_ZERO_ANGLE;
	
	gimbal->cmd.yaw_imu_tar = gimbal->info.rt_info.yaw_imu;
	gimbal->cmd.pitch_imu_tar = gimbal->info.rt_info.pitch_imu;
	
	if(fabs(half_cycle(gimbal->cmd.yaw_mec_tar - gimbal->yaw->rx_info->motor_angle,2*PI)) <= 0.01f && fabs(half_cycle(P_ZERO_ANGLE - D_Board_Rx_Info.pitch_mec,2*PI)) <= 0.01f)
	{
		Balance.Flag->Gimbal_Reset_OK = true;
	}
	
	return;
}

float yaw_tar = Y_ZERO_ANGLE;
 uint16_t U_time = 0;
 uint16_t reset_time = 0;
static bool last_u_flag = false;
void Gimbal_Mec_Update(Gimbal_t* gimbal)
{
	if(Balance.Flag->Mec_Flag == false && Balance.Flag->Rescue_Flag == false)
	{
		return;	
	}
	

	if(Balance.Flag->Rescue_Flag == true)
	{
		gimbal->cmd.yaw_mec_tar = Chassis.rescue_info->yaw_save_tar;
		gimbal->cmd.pitch_mec_tar = P_ZERO_ANGLE;
		
	}
	else
	{
		Gimbal_Reset_Init(gimbal);
	  
	 if(Balance.Flag->U_Turn_Flag == false && last_u_flag == false)
	 {
		 if(fabs(gimbal->misc.yaw_included_angle) <= PI/2)
	   {
		   gimbal->cmd.yaw_mec_tar = Y_ZERO_ANGLE;
	   }
	   else if(fabs(gimbal->misc.yaw_included_angle) > PI/2)
	   {
		   gimbal->cmd.yaw_mec_tar = gimbal->info.cfg_info.head_to[4];
	   }
	 }
	 else if(Balance.Flag->U_Turn_Flag == true && last_u_flag == false)
	 {
	   if(gimbal->cmd.yaw_mec_tar == gimbal->info.cfg_info.head_to[0])
		 {
			 gimbal->cmd.yaw_mec_tar = gimbal->info.cfg_info.head_to[4];
		 }
		 else if(gimbal->cmd.yaw_mec_tar == gimbal->info.cfg_info.head_to[4])
		 {
			 gimbal->cmd.yaw_mec_tar = gimbal->info.cfg_info.head_to[0];
		 }
	 }
   else if(Balance.Flag->U_Turn_Flag == true && last_u_flag == true)
	 {
		 U_time ++;
		 
	 	 if(gimbal->cmd.yaw_mec_tar == gimbal->info.cfg_info.head_to[0])
	 	 {
	 		 if(fabs(half_cycle(gimbal->info.cfg_info.head_to[0] - gimbal->yaw->rx_info->motor_angle,2*PI)) <= PI*5.f/180.f)
	 		 {
			 	 Balance.Flag->U_Turn_Flag = false;
				 U_time = 0;
			 }				
		 }
		 else if(gimbal->cmd.yaw_mec_tar == gimbal->info.cfg_info.head_to[4])
		 {
		 	if(fabs(half_cycle(gimbal->info.cfg_info.head_to[4] - gimbal->yaw->rx_info->motor_angle,2*PI)) <= PI*5.f/180.f)
		 	{
		 		Balance.Flag->U_Turn_Flag = false;
				U_time = 0;
		 	}
		 }
		 
		 if(U_time >= 600)
		 {
			 Balance.Flag->U_Turn_Flag = false;
			 U_time = 0;
		 }
	 }
		
    if(Balance.ctrl == RC_CTRL)
	  {
	    gimbal->cmd.pitch_mec_tar -= gimbal->info.cfg_info.rc_pitch_mec_k * rc_sensor.info->ch1 / 660.f;  //需要修改
	  }
	  else if(Balance.ctrl == KEY_CTRL)
	  {
		  gimbal->cmd.pitch_mec_tar -= gimbal->info.cfg_info.key_pitch_mec_k * rc_sensor.info->mouse_y;  //需要修改
	  }
	}

		
	gimbal->cmd.pitch_mec_tar = constrain(gimbal->cmd.pitch_mec_tar,P_MEC_ANGLE_MIN, P_MEC_ANGLE_MAX);
	gimbal->cmd.yaw_imu_tar = gimbal->info.rt_info.yaw_imu;
	gimbal->cmd.pitch_imu_tar = gimbal->info.rt_info.pitch_imu;
	
	
	last_u_flag = Balance.Flag->U_Turn_Flag;
}


void Gimbal_Gyro_Update(Gimbal_t* gimbal)
{
	
//	#ifndef VISION_TEST
//	if(Balance.Flag->Turn_Flag == false && Balance.Flag->Reserve_Fly_Flag == false && Balance.Flag->Imu_Flag == false && Balance.Flag->Test_Flag == false) 
//	{
//	  return;
//	}
//	#else 
	if(Balance.Flag->Turn_Flag == false && Balance.Flag->S_Turn_Flag == false && Balance.Flag->Reserve_Fly_Flag == false && Balance.Flag->Imu_Flag == false) 
	{
	  return;
	}
	
	if(Balance.Flag->chassis_reset == true)
	{
		reset_time ++;
		gimbal->cmd.yaw_mec_tar = gimbal->info.cfg_info.head_to[0];
		if(fabs(half_cycle(gimbal->yaw->rx_info->motor_angle - Y_ZERO_ANGLE,2*PI)) <= 5.f/180.f*PI)
		{
			Balance.Flag->chassis_reset = false;
			reset_time = 0;
		}
		else if(reset_time >= 1000)
		{
			Balance.Flag->chassis_reset = false;
			reset_time = 0;
		}
	}
	else if(Balance.Flag->chassis_reset == false)
	{
		if(Balance.Flag->U_Turn_Flag == false && last_u_flag == false)
	  {
		  if(fabs(gimbal->misc.yaw_included_angle) <= PI/2)
	    {
		    gimbal->cmd.yaw_mec_tar = gimbal->info.cfg_info.head_to[0];
	    }
	    else if(fabs(gimbal->misc.yaw_included_angle) > PI/2)
	    {
		    gimbal->cmd.yaw_mec_tar = gimbal->info.cfg_info.head_to[4];
	    }
	  }
	  else if(Balance.Flag->U_Turn_Flag == true && last_u_flag == false)
	  {
			gimbal->cmd.yaw_imu_tar += 180.f;
		
		  if(fabs(gimbal->cmd.yaw_imu_tar) > 180.f)
		  {
			  gimbal->cmd.yaw_imu_tar -= sgn(gimbal->cmd.yaw_imu_tar) * 360.f;
					
		  }
			
	  }
	  else if(Balance.Flag->U_Turn_Flag == true && last_u_flag == true)
	  {
			U_time ++;
		  if(fabs(half_cycle(gimbal->cmd.yaw_imu_tar - D_Board_Rx_Info.yaw_imu,360.f)) <= 5.f)
		  {
			  Balance.Flag->U_Turn_Flag = false;
				U_time = 0;
		  }
			if(U_time >= 600)
		  {
			   Balance.Flag->U_Turn_Flag = false;
			   U_time = 0;
		  }
	  }
		
		 
	}
  
//	if(D_Board_Tx_Pkt.vision_mode != 0)
//	{
//		gimbal->yaw->pid->gyro_pid.angle.out_max = 200.f;
//	}
//	else{
//	  gimbal->yaw->pid->gyro_pid.angle.out_max = 500.f;
//	
//	}

	
	if(Balance.ctrl == RC_CTRL)
	{
		gimbal->cmd.yaw_imu_tar -= gimbal->info.cfg_info.rc_yaw_gyro_k * rc_sensor.info->ch0 /660;
		gimbal->cmd.pitch_imu_tar += gimbal->info.cfg_info.rc_pitch_gyro_k * rc_sensor.info->ch1 /660;  //需要修改
	}
	else if(Balance.ctrl == KEY_CTRL)
	{
		gimbal->cmd.yaw_imu_tar -= gimbal->info.cfg_info.key_yaw_gyro_k * rc_sensor.info->mouse_x;
	  gimbal->cmd.pitch_imu_tar += gimbal->info.cfg_info.key_pitch_gyro_k * rc_sensor.info->mouse_y;  //需要修改
	}
		
	gimbal->cmd.yaw_imu_tar = half_cycle(gimbal->cmd.yaw_imu_tar, 360.f);
	gimbal->cmd.pitch_imu_tar = constrain(gimbal->cmd.pitch_imu_tar,P_GYRO_ANGLE_MIN, P_GYRO_ANGLE_MAX);
		
	gimbal->cmd.pitch_mec_tar = gimbal->info.rt_info.pitch_mec;
	
	last_u_flag = Balance.Flag->U_Turn_Flag;

}

void Vision_Self_Aim_Update(Gimbal_t* gimbal)
{
	if(Balance.Flag->Turn_Flag == false &&Balance.Flag->S_Turn_Flag == false && Balance.Flag->Reserve_Fly_Flag == false && Balance.Flag->Imu_Flag == false) 
	{
	  return;
	}
	
	if(D_Board_Tx_Pkt.vision_mode != 0 && D_Board_Rx_Info.vision_state == 1 && (D_Board_Rx_Info.is_find_Target == 1 || D_Board_Rx_Info.is_find_dafu == 1))
	{
		D_Board_Tx_Pkt.yaw_offset = D_Board_Rx_Info.vision_yaw_tar - D_Board_Rx_Info.yaw_imu;
		D_Board_Tx_Pkt.yaw_offset = half_cycle(D_Board_Tx_Pkt.yaw_offset,360.f);
	
		gimbal->cmd.yaw_imu_tar = D_Board_Rx_Info.vision_yaw_tar;
		gimbal->cmd.pitch_imu_tar = D_Board_Rx_Info.vision_pitch_tar;
		
		D_Board_Tx_Pkt.pitch_imu_tar = D_Board_Rx_Info.vision_pitch_tar;
	
	}
}

void Gimbal_Pid_Cal(Gimbal_t* gimbal)
{
	if(D_Board_Tx_Pkt.Gimbal_state == 0)
	{
		gimbal->yaw->tx_info->torque = 0;
		return;
	}
	
	if(Balance.Flag->Mec_Flag == true)
	{
		gimbal->yaw->pid->mec_pid.angle.target = gimbal->cmd.yaw_mec_tar;
		gimbal->yaw->pid->mec_pid.angle.measure = gimbal->yaw->rx_info->motor_angle;
		gimbal->yaw->pid->mec_pid.angle.err = gimbal->yaw->pid->mec_pid.angle.target - gimbal->yaw->pid->mec_pid.angle.measure;
		gimbal->yaw->pid->mec_pid.angle.err = half_cycle(gimbal->yaw->pid->mec_pid.angle.err, 2 * PI);
		
		single_pid_ctrl(&gimbal->yaw->pid->mec_pid.angle);
		
		gimbal->yaw->pid->mec_pid.speed.target = gimbal->yaw->pid->mec_pid.angle.out;
		gimbal->yaw->pid->mec_pid.speed.measure = gimbal->info.rt_info.yaw_v;
		gimbal->yaw->pid->mec_pid.speed.err = gimbal->yaw->pid->mec_pid.speed.target - gimbal->yaw->pid->mec_pid.speed.measure;
		
		single_pid_ctrl(&gimbal->yaw->pid->mec_pid.speed);
		
		gimbal->yaw->tx_info->torque = gimbal->yaw->pid->mec_pid.speed.out;
	}         
  
  else if(Balance.Flag->Imu_Flag == true || Balance.Flag->Turn_Flag == true || Balance.Flag->S_Turn_Flag == true)	
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
		gimbal->cmd.yaw_mec_tar = Y_ZERO_ANGLE;
		gimbal->cmd.pitch_mec_tar = P_ZERO_ANGLE;
		gimbal->cmd.pitch_imu_tar = 0;
		
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

