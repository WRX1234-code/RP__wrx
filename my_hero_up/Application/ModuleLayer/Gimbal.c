//#include "Chassis.h"
#include "gimbal.h"
#include "device.h"
#include "rp_math.h"
#include "can.h"
#include "motor.h"
#include "imu_sensor.h"
#include "rc_protocol.h"
#include "Communicate.h"

Gimbal_t gimbal_motor;

float Imu_Data_Contrary_Menage(float imu_data)
{
	float menage_data;
	if(imu_data>0)
	{
		menage_data=180-imu_data;
	}
	else if(imu_data<0)
	{
		menage_data=-180-imu_data;
	}
	return menage_data;
}

void Gyro_zero_bias(Gimbal_t* gimbal_motor)
{
	imu_sensor.work_state.err_code=IMU_DATA_CALI;
	gimbal_motor->zero_bias_flag=imu_sensor.work_state.err_code;
	
	imu_sensor.update(&imu_sensor);
	
	gimbal_motor->x_offset=imu_sensor.info->offset_info.gx_offset;
	gimbal_motor->y_offset=imu_sensor.info->offset_info.gy_offset;
	gimbal_motor->z_offset=imu_sensor.info->offset_info.gz_offset;
	
	imu_sensor.work_state.err_code=IMU_NONE_ERR;
}


void Gimbal_Init(Gimbal_t* gimbal_motor)
{
	gimbal_motor->gimbal_mode=1;
	
	gimbal_motor->gimbal_y_motor.y_motor=&kt_motor[0];
	
	gimbal_motor->gimbal_p_motor.p_mec=&gim_p_motor[MOTOR_MEC];
	gimbal_motor->gimbal_p_motor.p_gyro=&gim_p_motor[MOTOR_GYRO];
	
	gimbal_motor->gimbal_y_motor.y_motor->motor_all_pid.mec_pid.angle.target=Y_ZERO_ANGLE;
	gimbal_motor->gimbal_p_motor.p_mec->ctrl->angle_ctrl_outer->target=P_ZERO_ANGLE;
	
	gimbal_motor->gimbal_y_motor.rc_y_gyro_k=0.0001f;
	gimbal_motor->gimbal_p_motor.rc_p_mec_k=0.0002f;
	gimbal_motor->gimbal_p_motor.rc_p_gyro_k=0.00005f;
	
//	gimbal_motor->gimbal_y_motor.y_motor->tx_W_cmd(gimbal_motor->gimbal_y_motor.y_motor,MOTOR_RUN_ID);   
	
	Gyro_zero_bias(gimbal_motor);
	
	gimbal_motor->gimbal_y_motor.y_imu_angle=imu_sensor.info->base_info.yaw;
	gimbal_motor->gimbal_y_motor.y_imu_speed=-imu_sensor.info->base_info.rate_yaw;
	
	gimbal_motor->gimbal_p_motor.p_imu_angle=Imu_Data_Contrary_Menage(imu_sensor.info->base_info.roll);

	gimbal_motor->gimbal_p_motor.p_imu_speed=-imu_sensor.info->base_info.ave_rate_roll;
	
	gimbal_motor->gimbal_y_motor.y_motor->motor_all_pid.gyro_pid.angle.target=gimbal_motor->gimbal_y_motor.y_imu_angle;
	gimbal_motor->gimbal_p_motor.p_gyro->ctrl->angle_ctrl_outer->target=gimbal_motor->gimbal_p_motor.p_imu_angle;
	
}

void Gimbal_Remote_Receive(Gimbal_t* gimbal_motor)
{
	gimbal_motor->p_included_angle=(float)gimbal_motor->gimbal_p_motor.p_gyro->rx_info->encoder-P_ZERO_ANGLE;
	gimbal_motor->p_included_angle=motor_half_cycle(gimbal_motor->p_included_angle,8192);
	
	if(gimbal_motor->gimbal_mode==1)
	{
		gimbal_motor->gimbal_y_motor.y_motor->motor_all_pid.mec_pid.angle.target=Y_ZERO_ANGLE;
		
		gimbal_motor->gimbal_p_motor.p_mec->ctrl->angle_ctrl_outer->target-=rc_sensor.info->ch1*gimbal_motor->gimbal_p_motor.rc_p_mec_k;
	
		if(gimbal_motor->gimbal_p_motor.p_mec->ctrl->angle_ctrl_outer->target>P_MEC_ANGLE_MAX)
		{
			gimbal_motor->gimbal_p_motor.p_mec->ctrl->angle_ctrl_outer->target=P_MEC_ANGLE_MAX;
		}
		else if(gimbal_motor->gimbal_p_motor.p_mec->ctrl->angle_ctrl_outer->target<P_MEC_ANGLE_MIN)
		{
			gimbal_motor->gimbal_p_motor.p_mec->ctrl->angle_ctrl_outer->target=P_MEC_ANGLE_MIN;
		}
		
		gimbal_motor->gimbal_y_motor.y_imu_angle=imu_sensor.info->base_info.yaw;
		gimbal_motor->gimbal_p_motor.p_imu_angle=Imu_Data_Contrary_Menage(imu_sensor.info->base_info.roll);
		
		gimbal_motor->gimbal_y_motor.y_imu_speed=-imu_sensor.info->base_info.rate_yaw;
		gimbal_motor->gimbal_p_motor.p_imu_speed=-imu_sensor.info->base_info.ave_rate_roll;
		
		gimbal_motor->gimbal_y_motor.y_motor->motor_all_pid.gyro_pid.angle.target=gimbal_motor->gimbal_y_motor.y_imu_angle;
		gimbal_motor->gimbal_p_motor.p_gyro->ctrl->angle_ctrl_outer->target=gimbal_motor->gimbal_p_motor.p_imu_angle;
		
		
	}
	else if(gimbal_motor->gimbal_mode==2||gimbal_motor->gimbal_mode==3)
	{
		gimbal_motor->gimbal_y_motor.y_motor->motor_all_pid.gyro_pid.angle.target-=rc_sensor.info->ch0*gimbal_motor->gimbal_y_motor.rc_y_gyro_k;
		gimbal_motor->gimbal_y_motor.y_motor->motor_all_pid.gyro_pid.angle.target=motor_half_cycle(gimbal_motor->gimbal_y_motor.y_motor->motor_all_pid.gyro_pid.angle.target,360.f);
		
		gimbal_motor->gimbal_p_motor.p_gyro->ctrl->angle_ctrl_outer->target-=rc_sensor.info->ch1*gimbal_motor->gimbal_p_motor.rc_p_gyro_k;
		gimbal_motor->gimbal_p_motor.p_imu_angle=Imu_Data_Contrary_Menage(imu_sensor.info->base_info.roll);
		gimbal_motor->gimbal_p_motor.p_imu_speed=-imu_sensor.info->base_info.ave_rate_roll;
		
//		gimbal_motor->gimbal_p_motor.p_gyro->ctrl->angle_ctrl_outer->target=Imu_Data_Contrary_Menage(gimbal_motor->gimbal_p_motor.p_gyro->ctrl->angle_ctrl_outer->target);
		
		
		if(gimbal_motor->gimbal_p_motor.p_gyro->ctrl->angle_ctrl_outer->target>P_GYRO_ANGLE_MAX)
		{
			gimbal_motor->gimbal_p_motor.p_gyro->ctrl->angle_ctrl_outer->target=P_GYRO_ANGLE_MAX;
		}
		else if(gimbal_motor->gimbal_p_motor.p_gyro->ctrl->angle_ctrl_outer->target<P_GYRO_ANGLE_MIN)
		{
			gimbal_motor->gimbal_p_motor.p_gyro->ctrl->angle_ctrl_outer->target=P_GYRO_ANGLE_MIN;
		}
	}
}

void Gimbal_Send(Gimbal_t* gimbal_motor)
{
//	int16_t t;
//	uint8_t p_send_data[8];
	
	if(gimbal_heart_state==1)
	{
		Gimbal_Sleep(gimbal_motor);
	}
	else if(gimbal_heart_state==0)
	{
		if(gimbal_motor->gimbal_mode==1)
	  {
		  gimbal_motor->gimbal_y_motor.y_motor->KT_motor_info.tx_info.iqControl=gimbal_motor->gimbal_y_motor.y_motor->motor_all_pid.mec_pid.speed.out;
			
      gimbal_motor->gimbal_p_motor.p_mec->tx_info->torque=gimbal_motor->gimbal_p_motor.p_mec->ctrl->angle_ctrl_inner->out;
//		  t=gimbal_motor->gimbal_p_motor.p_mec->tx_info->torque;
	  	
	  }
		else if(gimbal_motor->gimbal_mode==2||gimbal_motor->gimbal_mode==3)
	  {
		  gimbal_motor->gimbal_y_motor.y_motor->KT_motor_info.tx_info.iqControl=gimbal_motor->gimbal_y_motor.y_motor->motor_all_pid.gyro_pid.speed.out;
			
	    gimbal_motor->gimbal_p_motor.p_gyro->tx_info->torque=gimbal_motor->gimbal_p_motor.p_gyro->ctrl->angle_ctrl_inner->out;
			
//			t=gimbal_motor->gimbal_p_motor.p_gyro->tx_info->torque;
	  }
		gimbal_motor->gimbal_y_motor.y_motor->tx_W_cmd(gimbal_motor->gimbal_y_motor.y_motor,TORQUE_CLOSE_LOOP_ID);
		
//		p_send_data[0]=0x00;
//	  p_send_data[1]=0x00;
//		p_send_data[2]=t>>8;
//	  p_send_data[3]=t;
//		p_send_data[4]=0x00;
//	  p_send_data[5]=0x00;
//	  p_send_data[6]=0x00;
//	  p_send_data[7]=0x00;
//		
//		CAN2_SendData(0x1FF,p_send_data);
	}
}
	

void Gimbal_PID_Calculate(Gimbal_t* gimbal_motor)
{
	gimbal_motor->gimbal_y_motor.y_imu_angle=imu_sensor.info->base_info.yaw;    
  gimbal_motor->gimbal_p_motor.p_imu_angle=Imu_Data_Contrary_Menage(imu_sensor.info->base_info.roll);  

  gimbal_motor->gimbal_y_motor.y_imu_speed=-imu_sensor.info->base_info.rate_yaw;   
  gimbal_motor->gimbal_p_motor.p_imu_speed=-imu_sensor.info->base_info.ave_rate_roll; 
	
	if(gimbal_motor->gimbal_mode==1)
	{
		
		gimbal_motor->gimbal_y_motor.y_motor->motor_all_pid.mec_pid.angle.measure=gimbal_motor->gimbal_y_motor.y_motor->KT_motor_info.rx_info.encoder;
		gimbal_motor->gimbal_y_motor.y_motor->motor_all_pid.mec_pid.angle.err=gimbal_motor->gimbal_y_motor.y_motor->motor_all_pid.mec_pid.angle.target-gimbal_motor->gimbal_y_motor.y_motor->motor_all_pid.mec_pid.angle.measure;
		gimbal_motor->gimbal_y_motor.y_motor->motor_all_pid.mec_pid.angle.err=motor_half_cycle(gimbal_motor->gimbal_y_motor.y_motor->motor_all_pid.mec_pid.angle.err,32768);
		
		single_pid_ctrl(&gimbal_motor->gimbal_y_motor.y_motor->motor_all_pid.mec_pid.angle);
		
    gimbal_motor->gimbal_y_motor.y_motor->motor_all_pid.mec_pid.speed.target=gimbal_motor->gimbal_y_motor.y_motor->motor_all_pid.mec_pid.angle.out;
		gimbal_motor->gimbal_y_motor.y_motor->motor_all_pid.mec_pid.speed.measure=gimbal_motor->gimbal_y_motor.y_imu_speed;
		gimbal_motor->gimbal_y_motor.y_motor->motor_all_pid.mec_pid.speed.err=gimbal_motor->gimbal_y_motor.y_motor->motor_all_pid.mec_pid.speed.target-gimbal_motor->gimbal_y_motor.y_motor->motor_all_pid.mec_pid.speed.measure;
		
		single_pid_ctrl(&gimbal_motor->gimbal_y_motor.y_motor->motor_all_pid.mec_pid.speed);
		
		
		gimbal_motor->gimbal_p_motor.p_mec->ctrl->angle_ctrl_outer->measure=gimbal_motor->gimbal_p_motor.p_mec->rx_info->encoder;
		gimbal_motor->gimbal_p_motor.p_mec->ctrl->angle_ctrl_outer->err=gimbal_motor->gimbal_p_motor.p_mec->ctrl->angle_ctrl_outer->target-gimbal_motor->gimbal_p_motor.p_mec->ctrl->angle_ctrl_outer->measure;
		gimbal_motor->gimbal_p_motor.p_mec->ctrl->angle_ctrl_outer->err=motor_half_cycle(gimbal_motor->gimbal_p_motor.p_mec->ctrl->angle_ctrl_outer->err,8192);
		
		single_pid_ctrl(gimbal_motor->gimbal_p_motor.p_mec->ctrl->angle_ctrl_outer);
		
		gimbal_motor->gimbal_p_motor.p_mec->ctrl->angle_ctrl_inner->target=gimbal_motor->gimbal_p_motor.p_mec->ctrl->angle_ctrl_outer->out;
		gimbal_motor->gimbal_p_motor.p_mec->ctrl->angle_ctrl_inner->measure=gimbal_motor->gimbal_p_motor.p_imu_speed;
		gimbal_motor->gimbal_p_motor.p_mec->ctrl->angle_ctrl_inner->err=gimbal_motor->gimbal_p_motor.p_mec->ctrl->angle_ctrl_inner->target-gimbal_motor->gimbal_p_motor.p_mec->ctrl->angle_ctrl_inner->measure;
		
		single_pid_ctrl(gimbal_motor->gimbal_p_motor.p_mec->ctrl->angle_ctrl_inner);
								 
	}
	else if(gimbal_motor->gimbal_mode==2||gimbal_motor->gimbal_mode==3)
	{
	  gimbal_motor->gimbal_y_motor.y_motor->motor_all_pid.gyro_pid.angle.measure=gimbal_motor->gimbal_y_motor.y_imu_angle;
		gimbal_motor->gimbal_y_motor.y_motor->motor_all_pid.gyro_pid.angle.err=gimbal_motor->gimbal_y_motor.y_motor->motor_all_pid.gyro_pid.angle.target-gimbal_motor->gimbal_y_motor.y_motor->motor_all_pid.gyro_pid.angle.measure;
		gimbal_motor->gimbal_y_motor.y_motor->motor_all_pid.gyro_pid.angle.err=motor_half_cycle(gimbal_motor->gimbal_y_motor.y_motor->motor_all_pid.gyro_pid.angle.err,360);
		
		single_pid_ctrl(&gimbal_motor->gimbal_y_motor.y_motor->motor_all_pid.gyro_pid.angle);
	
		gimbal_motor->gimbal_y_motor.y_motor->motor_all_pid.gyro_pid.speed.target=gimbal_motor->gimbal_y_motor.y_motor->motor_all_pid.gyro_pid.angle.out;
	  gimbal_motor->gimbal_y_motor.y_motor->motor_all_pid.gyro_pid.speed.measure=gimbal_motor->gimbal_y_motor.y_imu_speed+gimbal_motor->z_offset;
		gimbal_motor->gimbal_y_motor.y_motor->motor_all_pid.gyro_pid.speed.err=gimbal_motor->gimbal_y_motor.y_motor->motor_all_pid.gyro_pid.speed.target-gimbal_motor->gimbal_y_motor.y_motor->motor_all_pid.gyro_pid.speed.measure;
		
		single_pid_ctrl(&gimbal_motor->gimbal_y_motor.y_motor->motor_all_pid.gyro_pid.speed);
		
		
		gimbal_motor->gimbal_p_motor.p_gyro->ctrl->angle_ctrl_outer->measure=gimbal_motor->gimbal_p_motor.p_imu_angle;
		gimbal_motor->gimbal_p_motor.p_gyro->ctrl->angle_ctrl_outer->err=gimbal_motor->gimbal_p_motor.p_gyro->ctrl->angle_ctrl_outer->target-gimbal_motor->gimbal_p_motor.p_gyro->ctrl->angle_ctrl_outer->measure;
		gimbal_motor->gimbal_p_motor.p_gyro->ctrl->angle_ctrl_outer->err=motor_half_cycle(gimbal_motor->gimbal_p_motor.p_gyro->ctrl->angle_ctrl_outer->err,360);
		
		single_pid_ctrl(gimbal_motor->gimbal_p_motor.p_gyro->ctrl->angle_ctrl_outer);
		
		gimbal_motor->gimbal_p_motor.p_gyro->ctrl->angle_ctrl_inner->target=gimbal_motor->gimbal_p_motor.p_gyro->ctrl->angle_ctrl_outer->out;
		gimbal_motor->gimbal_p_motor.p_gyro->ctrl->angle_ctrl_inner->measure=gimbal_motor->gimbal_p_motor.p_imu_speed+gimbal_motor->y_offset;
		gimbal_motor->gimbal_p_motor.p_gyro->ctrl->angle_ctrl_inner->err=gimbal_motor->gimbal_p_motor.p_gyro->ctrl->angle_ctrl_inner->target-gimbal_motor->gimbal_p_motor.p_gyro->ctrl->angle_ctrl_inner->measure;
		
		single_pid_ctrl(gimbal_motor->gimbal_p_motor.p_gyro->ctrl->angle_ctrl_inner);
		
	}
		
}

void Gimbal_Heart_Beat(Gimbal_t* gimbal_motor)
{
	if(heart_cnt>70)
	{
		heart_cnt=70;
		communicate_chassis_target.heart_state=1;
		gimbal_heart_state=1;
	}
	else
	{
		communicate_chassis_target.heart_state=0;
		gimbal_heart_state=0;
	} 
}

void Gimbal_Sleep(Gimbal_t* gimbal_motor)
{
	gimbal_motor->gimbal_y_motor.y_motor->W_iqControl(gimbal_motor->gimbal_y_motor.y_motor,0);
	gimbal_motor->gimbal_y_motor.y_motor->tx_W_cmd(gimbal_motor->gimbal_y_motor.y_motor,TORQUE_CLOSE_LOOP_ID);
	gimbal_motor->gimbal_p_motor.p_mec->single_sleep(gimbal_motor->gimbal_p_motor.p_mec);
	gimbal_motor->gimbal_p_motor.p_gyro->single_sleep(gimbal_motor->gimbal_p_motor.p_gyro);
		
}

void Gimbal_Drive(Gimbal_t* gimbal_motor)
{
	
	Gimbal_Remote_Receive(gimbal_motor);
	Gimbal_PID_Calculate(gimbal_motor);
	Gimbal_Send(gimbal_motor);
	
}






