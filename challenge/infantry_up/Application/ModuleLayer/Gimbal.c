#include "Gimbal.h"
#include "Chassis.h"
#include "motor.h"
#include "imu_sensor.h"
#include "Robot.h"
#include "vision_protocol.h"
#include "Board_protocol.h"

Gimbal_t gimbal = {
	.pitch = &Pitch_Motor,
	
	.info = {
		.cal = {
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


/*------------------------------对内API定义------------------------------------*/

/**
  * @brief     陀螺仪数据处理，矫正方向 
	*/
static void Gimbal_Imu_data_Update(Gimbal_t* gimbal)
{
	gimbal->info.imu.yaw_angle = -imu_sensor.info->base_info.yaw;
  gimbal->info.imu.yaw_speed = -imu_sensor.info->base_info.rate_yaw;
  gimbal->info.imu.pitch_angle = -imu_sensor.info->base_info.pitch;
	gimbal->info.imu.pitch_speed = -imu_sensor.info->base_info.ave_rate_pitch;
	C_Board_Tx_Pkt.pitch_imu = gimbal->info.imu.pitch_angle;
	C_Board_Tx_Pkt.yaw_imu = gimbal->info.imu.yaw_angle;
	C_Board_Tx_Pkt.pitch_v = gimbal->info.imu.pitch_speed;
	C_Board_Tx_Pkt.yaw_v = gimbal->info.imu.yaw_speed;
	
	
}


/**	
  * @brief    yaw轴零偏校准方法1
  * @note     校准角度
  */	
static void Gyro_bias_manage(Gimbal_t* gimbal)
{
	gimbal->info.imu.yaw_angle += gimbal->info.cal.yaw_bias_add;    //需配置零偏角度
                                                                                                                   
	while (abs(gimbal->info.imu.yaw_angle) > (360 / 2))//可能卡死                                         
  {                                                                                                             
	  gimbal->info.imu.yaw_angle= motor_half_cycle(gimbal->info.imu.yaw_angle,360.f);      
	}                                                                                                                                                        

}


/*------------------------------------对外API定义--------------------------------------*/

/**	
  * @brief    yaw轴零偏校准方法2
  * @note     校准速度，可以尝试
  */	
void Gyro_zero_bias(Gimbal_t* gimbal)
{

		imu_sensor.work_state.err_code=IMU_DATA_CALI;       //该标志位用于激活imu_sensor.update内部自行零偏校准
		
		imu_sensor.info->offset_info.gx_offset = 0.0f;
		imu_sensor.info->offset_info.gy_offset = 0.0f;
		imu_sensor.info->offset_info.gz_offset = 0.0f;
		
		imu_sensor.update(&imu_sensor);
	
	  gimbal->flag.zero_bias_flag = 1;
		
	  gimbal->misc.x_offset=imu_sensor.info->offset_info.gx_offset;     //记录零偏相关数值，便于查看零偏情况
	  gimbal->misc.y_offset=imu_sensor.info->offset_info.gy_offset;
	  gimbal->misc.z_offset=imu_sensor.info->offset_info.gz_offset;
	
}


/**	
  * @brief    机械模式状态更新
  * @note     
  */
void Gimbal_Mec_Update(Gimbal_t* gimbal)
{
  gimbal->cmd.pitch.mec_angle_target = C_Board_Rx_Info.pitch_mec_tar;
	
	gimbal->cmd.pitch.mec_angle_target = constrain(gimbal->cmd.pitch.mec_angle_target ,P_MEC_ANGLE_MIN ,P_MEC_ANGLE_MAX);   //限幅 
	
	//实时更新陀螺仪目标角度，确保切换模式头不动                                
	gimbal->cmd.pitch.gyro_angle_target = gimbal->info.imu.pitch_angle;                    
  C_Board_Tx_Pkt.pitch_mec = gimbal->cmd.pitch.mec_angle_target; 
}	
	

/**	
  * @brief    陀螺仪模式状态更新
  * @note     
  */
void Gimbal_Gyro_Update(Gimbal_t* gimbal)
{
	gimbal->misc.pitch_included_angle = gimbal->pitch->rx_info->motor_angle - P_ZERO_ANGLE;//计算pitch轴相对角度
	gimbal->misc.pitch_included_angle = motor_half_cycle(gimbal->misc.pitch_included_angle,3.1415f * 2);
  
	gimbal->cmd.pitch.gyro_angle_target = C_Board_Rx_Info.pitch_imu_tar;
	
//	gimbal->cmd.pitch.gyro_angle_target = constrain(gimbal->cmd.pitch.gyro_angle_target , P_GYRO_ANGLE_MIN , P_GYRO_ANGLE_MAX);
	
	gimbal->cmd.pitch.mec_angle_target = gimbal->pitch->rx_info->motor_angle;
	
}	
	
void Gimbal_Self_Aim_Update(Gimbal_t* gimbal)
{
	if(C_Board_Rx_Info.vision_mode == 0)
	{
		return;
	}
	else if(C_Board_Rx_Info.vision_mode == 1)
	{
		gimbal->cmd.pitch.gyro_angle_target = vision_rx_frame.pitch;
		gimbal->cmd.yaw.gyro_angle_target = vision_rx_frame.yaw;
	}
}

/**	
  * @brief    视觉所需数据更新
  * @note     
  */
void Gimbal_To_Vision_Update(Gimbal_t* gimbal)
{
	/*需要修改*/ 
	
	vision_tx_frame.pitch = 0;
	vision_tx_frame.yaw = 0;
	vision_tx_frame.roll = 0;
	
	vision_tx_frame.pitch_speed = 0;
	vision_tx_frame.yaw_speed = 0;

	if(C_Board_Rx_Info.vision_mode == 0)
	{
		vision_tx_frame.yaw_offset = 0;
	  vision_tx_frame.pitch_offset = 0;
	}
	else if(C_Board_Rx_Info.vision_mode == 1)
	{
		vision_tx_frame.yaw_offset = C_Board_Rx_Info.yaw_offset;
	  vision_tx_frame.pitch_offset = gimbal->pitch->rx_info->motor_angle - gimbal->cmd.pitch.gyro_angle_target;
	}
  
}

/**	
  * @brief    云台输出值更新
  * @note     yaw轴电机在底盘，此处不作PID计算
  */
void Gimbal_Send(Gimbal_t* gimbal)
{
	
	//储存输出值，便于查看
	if(C_Board_Rx_Info.Gimbal_mode == 2 || C_Board_Rx_Info.Gimbal_mode == 3)
	{
	  gimbal->cmd.pitch.output = gimbal->pitch->pid->mec_pid.speed.out;
	}
	else if(C_Board_Rx_Info.Gimbal_mode == 0 || C_Board_Rx_Info.Gimbal_mode == 1)
	{
		gimbal->cmd.pitch.output = gimbal->pitch->pid->gyro_pid.speed.out;
	}
	 
	gimbal->pitch->tx_info->torque = gimbal->cmd.pitch.output;
	gimbal->pitch->single_set_torque(gimbal->pitch);
}


/**	
  * @brief    云台PID计算
  * @note     其实只有pitch的PID计算
  */
void Gimbal_PID_Cal(Gimbal_t* gimbal)
{
	if(robot.state == LOST)
	{
		return;
	}
	
	if(C_Board_Rx_Info.Gimbal_mode == 2 || C_Board_Rx_Info.Gimbal_mode == 3)
	{
		gimbal->pitch->pid->mec_pid.angle.target = gimbal->cmd.pitch.mec_angle_target;
		gimbal->pitch->pid->mec_pid.angle.measure = gimbal->pitch->rx_info->motor_angle;
		gimbal->pitch->pid->mec_pid.angle.err = gimbal->pitch->pid->mec_pid.angle.target - gimbal->pitch->pid->mec_pid.angle.measure;
		gimbal->pitch->pid->mec_pid.angle.err = motor_half_cycle(gimbal->pitch->pid->mec_pid.angle.err, 3.1415f * 2);
		
		single_pid_ctrl(&gimbal->pitch->pid->mec_pid.angle);   //内含滤波限幅
		
    gimbal->pitch->pid->mec_pid.speed.target = gimbal->pitch->pid->mec_pid.angle.out;
		gimbal->pitch->pid->mec_pid.speed.measure = - gimbal->info.imu.pitch_speed;
		gimbal->pitch->pid->mec_pid.speed.err = gimbal->pitch->pid->mec_pid.speed.target - gimbal->pitch->pid->mec_pid.speed.measure;
		
		single_pid_ctrl(&gimbal->pitch->pid->mec_pid.speed);
								 
	}
	else if(C_Board_Rx_Info.Gimbal_mode == 0 || C_Board_Rx_Info.Gimbal_mode == 1)
	{
		gimbal->pitch->pid->gyro_pid.angle.target = gimbal->cmd.pitch.gyro_angle_target;
		gimbal->pitch->pid->gyro_pid.angle.measure = gimbal->info.imu.pitch_angle;
		gimbal->pitch->pid->gyro_pid.angle.err = gimbal->pitch->pid->gyro_pid.angle.target - gimbal->pitch->pid->gyro_pid.angle.measure;
		gimbal->pitch->pid->gyro_pid.angle.err = motor_half_cycle(gimbal->pitch->pid->gyro_pid.angle.err, 360.f);
		
		single_pid_ctrl(&gimbal->pitch->pid->gyro_pid.angle);
		
    gimbal->pitch->pid->gyro_pid.speed.target = gimbal->pitch->pid->gyro_pid.angle.out;
		gimbal->pitch->pid->gyro_pid.speed.measure = - gimbal->info.imu.pitch_speed;
		gimbal->pitch->pid->gyro_pid.speed.err = gimbal->pitch->pid->gyro_pid.speed.target - gimbal->pitch->pid->gyro_pid.speed.measure;
		
		single_pid_ctrl(&gimbal->pitch->pid->gyro_pid.speed);
	}
	
}


/**	
  * @brief    云台睡眠
  */
void Gimbal_Sleep(Gimbal_t* gimbal)
{
	gimbal->cmd.pitch.output = 0;
	
	//初始化各方向角度目标，但无论什么模式开控后是先机械归位再转回原模式
	gimbal->cmd.pitch.mec_angle_target = P_ZERO_ANGLE;

	gimbal->pitch->single_sleep(gimbal->pitch);	
}


/**	
  * @brief    云台总控
  */
void Gimbal_Work(Gimbal_t* gimbal)
{
	Gimbal_Imu_data_Update(gimbal);
	Gyro_bias_manage(gimbal);   //可循环调用，另一个尽量在静止时条件调用

	switch (C_Board_Rx_Info.Gimbal_state)
	{
		case 0:              //掉线	
			Gimbal_Sleep(gimbal);
		  break;
		
		case 1:              //在线
			switch (C_Board_Rx_Info.Gimbal_mode)
			{
				case 2:
					Gimbal_Mec_Update(gimbal);     //机械模式更新
		
					break;
					
				case 3:                          //吊射模式更新
					Gimbal_Mec_Update(gimbal);    
					break;
						
				case 0:
				case 1:                          //对云台来说，普通小陀螺跟普通陀螺操作逻辑一样
					Gimbal_Gyro_Update(gimbal);    //陀螺仪模式更新
				
					if(C_Board_Rx_Info.vision_mode == 1)
					{
						Gimbal_Self_Aim_Update(gimbal);
					}
					break;	
					
				default:
					break;
		   }	
			
			Gimbal_PID_Cal(gimbal);
//      Gimbal_Send(gimbal);	
			 break;
			 
		default:
		 break;
	
	}			
}
