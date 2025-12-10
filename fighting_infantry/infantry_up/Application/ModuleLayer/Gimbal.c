#include "Gimbal.h"
#include "Chassis.h"
#include "motor.h"
#include "imu_sensor.h"
#include "Robot.h"


Gimbal_t gimbal = {
	.pitch = &Pitch_Motor,
	
	.info = {
		.cal = {
			.rc_yaw_mec_k = 0,
		  .rc_yaw_gyro_k = 0,
		  .rc_pitch_mec_k = 0,
			.rc_pitch_gyro_k = 0,
		  .key_yaw_mec_k = 0,
		  .key_yaw_gyro_k = 0,
			.key_pitch_mec_k = 0,
			.key_pitch_gyro_k = 0,
			.yaw_bias_add = 0,
		},             
	},     
	
  .cmd = {
		.yaw = {
			.mec_angle_target = Y_ZERO_ANGLE,
		},
		.pitch = {
			.mec_angle_target = P_ZERO_ANGLE,
		},
	},	
	
  .flag = {
	  .zero_bias_flag =0,
	  .init_zero_flag =0,
	},          
                      
};

static void Gimbal_Imu_data_Update(Gimbal_t* gimbal)
{
	gimbal->info.imu.yaw_angle = 0;
  gimbal->info.imu.yaw_speed = 0;
  gimbal->info.imu.pitch_angle = 0;
	gimbal->info.imu.pitch_speed = 0;
	
}


static void Gyro_zero_bias(Gimbal_t* gimbal)
{

		imu_sensor.work_state.err_code=IMU_DATA_CALI;
		
		imu_sensor.info->offset_info.gx_offset = 0.0f;
		imu_sensor.info->offset_info.gy_offset = 0.0f;
		imu_sensor.info->offset_info.gz_offset = 0.0f;
		
		imu_sensor.update(&imu_sensor);
	
	  gimbal->flag.zero_bias_flag = 1;
		
	  gimbal->misc.x_offset=imu_sensor.info->offset_info.gx_offset;
	  gimbal->misc.y_offset=imu_sensor.info->offset_info.gy_offset;
	  gimbal->misc.z_offset=imu_sensor.info->offset_info.gz_offset;
	
}

static void Gyro_bias_manage(Gimbal_t* gimbal)
{
	gimbal->info.imu.yaw_angle += gimbal->info.cal.yaw_bias_add;    //
                                                                                                                   
	while (abs(gimbal->info.imu.yaw_angle) > (360 / 2))//¿ÉÄÜ¿¨ËÀ                                         
  {                                                                                                             
	  gimbal->info.imu.yaw_angle= motor_half_cycle(gimbal->info.imu.yaw_angle,360.f);      
	}                                                                                                                                                        

}


void Gimbal_Mec_Update(Gimbal_t* gimbal)
{
	
	gimbal->misc.yaw_included_angle = (float)gimbal->pitch->rx_info->motor_angle - P_ZERO_ANGLE;
	
	gimbal->misc.pitch_included_angle = motor_half_cycle(gimbal->misc.pitch_included_angle,y_encoder_val_max - y_encoder_val_min + 1);
	
	gimbal->cmd.yaw.mec_angle_target = Y_ZERO_ANGLE;
		
	if(robot.CU == RC_CU)
	{
		gimbal->cmd.pitch.mec_angle_target += rc_sensor.info->ch1 * gimbal->info.cal.rc_pitch_mec_k;
	}
	else if(robot.CU == KEY_CU)
	{
		gimbal->cmd.pitch.mec_angle_target += rc_sensor.info->mouse_y * gimbal->info.cal.key_pitch_mec_k;
	}
	
	gimbal->cmd.pitch.mec_angle_target = constrain(gimbal->cmd.pitch.mec_angle_target , P_MEC_ANGLE_MIN , P_MEC_ANGLE_MAX);
	gimbal->cmd.pitch.gyro_angle_target = gimbal->info.imu.pitch_angle;

}	
	
void Gimbal_Gyro_Update(Gimbal_t* gimbal)
{
	
	if(robot.CU == RC_CU)
	{
		gimbal->cmd.yaw.gyro_angle_target -= rc_sensor.info->ch0*gimbal->info.cal.rc_yaw_gyro_k;
		gimbal->cmd.pitch.gyro_angle_target += rc_sensor.info->ch1 * gimbal->info.cal.rc_pitch_gyro_k;
	}
	else if(robot.CU == KEY_CU)
	{
		gimbal->cmd.yaw.gyro_angle_target -=rc_sensor.info->mouse_x*gimbal->info.cal.key_yaw_gyro_k;
		gimbal->cmd.pitch.gyro_angle_target += rc_sensor.info->mouse_y * gimbal->info.cal.key_pitch_gyro_k;
	}
	
	gimbal->cmd.pitch.gyro_angle_target = constrain(gimbal->cmd.pitch.mec_angle_target , P_GYRO_ANGLE_MIN , P_GYRO_ANGLE_MAX);
	gimbal->cmd.pitch.mec_angle_target = gimbal->pitch->rx_info->motor_angle;
	
}	
	
void Gimbal_Send(Gimbal_t* gimbal)
{
	if(robot.base_mode == MEC)
	{
	  gimbal->cmd.pitch.output = gimbal->pitch->pid->mec_pid.speed.out;
	}
	else if(robot.base_mode == GYRO || robot.base_mode == S_GYRO)
	{
		gimbal->cmd.pitch.output = gimbal->pitch->pid->gyro_pid.speed.out;
	}
	 
}
	
void Gimbal_PID_Cal(Gimbal_t* gimbal)
{
	
	if(robot.base_mode == MEC)
	{
		gimbal->pitch->pid->mec_pid.angle.target = gimbal->cmd.pitch.mec_angle_target;
		gimbal->pitch->pid->mec_pid.angle.measure = gimbal->pitch->rx_info->motor_angle;
		gimbal->pitch->pid->mec_pid.angle.err = gimbal->pitch->pid->mec_pid.angle.target - gimbal->pitch->pid->mec_pid.angle.measure;
		
		
		single_pid_ctrl(&gimbal->pitch->pid->mec_pid.angle);
		
    gimbal->pitch->pid->mec_pid.speed.target = gimbal->pitch->pid->mec_pid.angle.out;
		gimbal->pitch->pid->mec_pid.speed.measure = gimbal->info.imu.pitch_speed;
		gimbal->pitch->pid->mec_pid.speed.err = gimbal->pitch->pid->mec_pid.speed.target - gimbal->pitch->pid->mec_pid.speed.measure;
		
		single_pid_ctrl(&gimbal->pitch->pid->mec_pid.speed);
								 
	}
	else if(robot.base_mode == GYRO || robot.base_mode == S_GYRO)
	{
		gimbal->pitch->pid->gyro_pid.angle.target = gimbal->cmd.pitch.gyro_angle_target;
		gimbal->pitch->pid->gyro_pid.angle.measure = gimbal->info.imu.pitch_angle;
		gimbal->pitch->pid->gyro_pid.angle.err = gimbal->pitch->pid->gyro_pid.angle.target - gimbal->pitch->pid->gyro_pid.angle.measure;
		
		single_pid_ctrl(&gimbal->pitch->pid->gyro_pid.angle);
		
    gimbal->pitch->pid->gyro_pid.speed.target = gimbal->pitch->pid->gyro_pid.angle.out;
		gimbal->pitch->pid->gyro_pid.speed.measure = gimbal->info.imu.pitch_speed;
		gimbal->pitch->pid->gyro_pid.speed.err = gimbal->pitch->pid->gyro_pid.speed.target - gimbal->pitch->pid->gyro_pid.speed.measure;
		
		single_pid_ctrl(&gimbal->pitch->pid->gyro_pid.speed);
	}
	
}

void Gimbal_Sleep(Gimbal_t* gimbal)
{
	gimbal->cmd.yaw.output = 0;
	gimbal->cmd.pitch.output = 0;
	
	gimbal->cmd.yaw.mec_angle_target = Y_ZERO_ANGLE;
	gimbal->cmd.pitch.mec_angle_target = P_ZERO_ANGLE;
	
	gimbal->cmd.yaw.gyro_angle_target = gimbal->info.imu.yaw_angle;
	gimbal->cmd.pitch.gyro_angle_target = gimbal->info.imu.pitch_angle;
		
}

void Gimbal_Work(Gimbal_t* gimbal)
{
	Gyro_bias_manage(gimbal);

	switch (robot.state)
	{
		case OFFLINE:
			Gimbal_Sleep(gimbal);
		  break;
		
		case ONLINE:
			switch (robot.base_mode)
			{
				case MEC:
					Gimbal_Mec_Update(gimbal);
					switch (robot.adv_mode)
					{
						case SUSPEND:
							break;
						
						case NO_ADV_MODE:
							break;
						
						default:
							break;
					}
					break;
				
				case GYRO:
				case S_GYRO:
					Gimbal_Gyro_Update(gimbal);
					switch (robot.adv_mode)
					{
						case H_S_S_GYRO:
							break;
						case SELF_AIM:
							break;
						
						case MELEE:
							break;
						
						default:
							break;
					}
					break;
		  }		
	}			
	Gimbal_PID_Cal(gimbal);
  Gimbal_Send(gimbal);		
	
}
