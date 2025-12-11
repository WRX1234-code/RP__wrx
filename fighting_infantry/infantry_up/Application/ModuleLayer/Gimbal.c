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


/*------------------------------对内API定义------------------------------------*/

/**
  * @brief     陀螺仪数据处理，矫正方向 
	*/
static void Gimbal_Imu_data_Update(Gimbal_t* gimbal)
{
	gimbal->info.imu.yaw_angle = 0;
  gimbal->info.imu.yaw_speed = 0;
  gimbal->info.imu.pitch_angle = 0;
	gimbal->info.imu.pitch_speed = 0;
	
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
	
	gimbal->misc.yaw_included_angle = (float)gimbal->pitch->rx_info->motor_angle - P_ZERO_ANGLE;     //计算yaw轴相对角度
	
	gimbal->misc.pitch_included_angle = motor_half_cycle(gimbal->misc.pitch_included_angle,y_encoder_val_max - y_encoder_val_min + 1);  //计算pitch轴相对角度
	
	gimbal->cmd.yaw.mec_angle_target = Y_ZERO_ANGLE;      
		
	if(robot.CU == RC_CU)            //遥控器操控
	{
		gimbal->cmd.pitch.mec_angle_target += rc_sensor.info->ch1 * gimbal->info.cal.rc_pitch_mec_k;
	}
	else if(robot.CU == KEY_CU)          //键鼠操控
	{
		gimbal->cmd.pitch.mec_angle_target += rc_sensor.info->mouse_y * gimbal->info.cal.key_pitch_mec_k;
	}
	
	gimbal->cmd.pitch.mec_angle_target = constrain(gimbal->cmd.pitch.mec_angle_target , P_MEC_ANGLE_MIN , P_MEC_ANGLE_MAX);   //限幅
	
	//实时更新陀螺仪目标角度，确保切换模式头不动
	gimbal->cmd.yaw.gyro_angle_target = gimbal->info.imu.yaw_angle;                                 
	gimbal->cmd.pitch.gyro_angle_target = gimbal->info.imu.pitch_angle;                    

}	
	

/**	
  * @brief    陀螺仪模式状态更新
  * @note     
  */
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
	

/**	
  * @brief    云台输出值更新
  * @note     yaw轴电机在底盘，此处不作PID计算
  */
void Gimbal_Send(Gimbal_t* gimbal)
{
	
	//储存输出值，便于查看
	if(robot.base_mode == MEC)
	{
	  gimbal->cmd.pitch.output = gimbal->pitch->pid->mec_pid.speed.out;
	}
	else if(robot.base_mode == GYRO || robot.base_mode == S_GYRO)
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
	if(robot.state == ONLINE)
	{
		
	}
	if(robot.base_mode == MEC)
	{
		gimbal->pitch->pid->mec_pid.angle.target = gimbal->cmd.pitch.mec_angle_target;
		gimbal->pitch->pid->mec_pid.angle.measure = gimbal->pitch->rx_info->motor_angle;
		gimbal->pitch->pid->mec_pid.angle.err = gimbal->pitch->pid->mec_pid.angle.target - gimbal->pitch->pid->mec_pid.angle.measure;
		
		
		single_pid_ctrl(&gimbal->pitch->pid->mec_pid.angle);   //内含滤波限幅
		
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


/**	
  * @brief    云台睡眠
  */
void Gimbal_Sleep(Gimbal_t* gimbal)
{
	gimbal->cmd.yaw.output = 0;
	gimbal->cmd.pitch.output = 0;
	
	//初始化各方向角度目标，但无论什么模式开控后是先机械归位再转回原模式
	gimbal->cmd.yaw.mec_angle_target = Y_ZERO_ANGLE;
	gimbal->cmd.pitch.mec_angle_target = P_ZERO_ANGLE;
	
	gimbal->cmd.yaw.gyro_angle_target = gimbal->info.imu.yaw_angle;
	gimbal->cmd.pitch.gyro_angle_target = gimbal->info.imu.pitch_angle;
		
}


/**	
  * @brief    云台总控
  */
void Gimbal_Work(Gimbal_t* gimbal)
{
	Gyro_bias_manage(gimbal);   //可循环调用，另一个尽量在静止时条件调用

	switch (robot.state)
	{
		case OFFLINE:             //掉线，需要优化
			Gimbal_Sleep(gimbal);
		  break;
		
		case ONLINE:              //在线
			switch (robot.base_mode)
			{
				case MEC:
					Gimbal_Mec_Update(gimbal);     //机械模式更新
					switch (robot.adv_mode)
					{
						case SUSPEND:                //吊射模式更新
							break;
						
						case NO_ADV_MODE:
							break;
						
						default:
							break;
					}
					break;
				
				case GYRO:
				case S_GYRO:                     //对云台来说，普通小陀螺跟普通陀螺操作逻辑一样
					Gimbal_Gyro_Update(gimbal);    //陀螺仪模式更新
					switch (robot.adv_mode)
					{
						case H_S_S_GYRO:             //高速小陀螺
							break;
						case SELF_AIM:               //自瞄
							break;
						
						case MELEE:                  //近战
							break; 
						
						default:
							break;
					}
					break;
					
				default:
					break;
		   }	
			
			Gimbal_PID_Cal(gimbal);
      Gimbal_Send(gimbal);	
			 
		default:
		 break;
	
	}			
}
