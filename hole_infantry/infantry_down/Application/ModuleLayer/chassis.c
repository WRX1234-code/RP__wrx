#include "chassis.h"
#include "infantry.h"

Chassis_t  chassis = {
	.wheel[WHEEL_RF] = &wheel_motor[WHEEL_RF],
	.wheel[WHEEL_RB] = &wheel_motor[WHEEL_RB],
	.wheel[WHEEL_LF] = &wheel_motor[WHEEL_LF],
	.wheel[WHEEL_LB] = &wheel_motor[WHEEL_LB],
	
	.pid_mode = SPEED_MODE,
	
	
};


static void Chassis_Target_Update(Chassis_t* chassis)
{
	int16_t yaw_angle_err;
	float yaw_angle_err_rad;
	
	int16_t front_speed = rc_sensor.info->ch3/660.f * FRONT_SPEED_MAX;
  int16_t right_speed = rc_sensor.info->ch2/660.f * RIGHT_SPEED_MAX;
	int16_t cycle_speed = rc_sensor.info->ch0/660.f * CYCLE_SPEED_MAX;
	switch (infantry.mode)
	{
	  case C_SLEEP:
		case C_INIT:
      chassis->target.front_speed = 0;
		  chassis->target.right_speed = 0;
	    chassis->target.cycle_speed = 0;
	
	    break;
		
		case C_BOSS:
			chassis->target.front_speed = front_speed;
		  chassis->target.right_speed = right_speed;
			if(infantry.flag.turn_flag == false)
			{
	      chassis->target.cycle_speed = cycle_speed;
			}
			else{
			 chassis->target.cycle_speed = -TURN_CYCLE_SPEED; // 小陀螺速度
			}
			break;
		
		case C_SLAVE:
      
				//	if (abs(gimbal.base_info.yaw_motor_angle) > 16384) 
        //	{
        //		front *= -1;
        //		right *= -1;
        //	}
	
        //	yaw_angle_err = gimbal.base_info.yaw_motor_angle / 32768.f * 4096.f; // yaw轴   相对底盘   角度(-4096~4096)(顺时针为正)
        //	float yaw_angle_err_rad = (double)yaw_angle_err / 4096.f * 3.14159;	 // yaw轴角度转弧度制（-π~π）

	      // front和right值计算
	      chassis->target.front_speed = front_speed * cos(yaw_angle_err_rad) - right_speed * sin(yaw_angle_err_rad);
	      chassis->target.right_speed = right_speed * cos(yaw_angle_err_rad) + front_speed * sin(yaw_angle_err_rad);
	
	      chassis->target.cycle_speed = yaw_angle_err_rad * yaw_angle_err_rad*sgn(yaw_angle_err_rad) /4096.f*30;
			
			break;
		
		
		default:
			break;
	}

}




static void Chassis_Speed_Calculate(Chassis_t* chassis)
{
	int16_t front = chassis->target.front_speed;
	int16_t right = chassis->target.right_speed;
	int16_t cycle = chassis->target.cycle_speed;
	
	int16_t speed_sum;
	float K;
	
	speed_sum = abs(front) + abs(right) + abs(cycle);
	
	if(speed_sum > CHASSIS_SPEED_MAX)
	{
		K = (float)CHASSIS_SPEED_MAX / (float)speed_sum;
	}
	else 
	{
		K = 1;
	}

	front *= K;
	right *= K;
	cycle *= K;
	
	chassis->target.motor_speed[WHEEL_LF]  =   front + right + cycle; 
	chassis->target.motor_speed[WHEEL_LB]  =   front - right + cycle;
	chassis->target.motor_speed[WHEEL_RF]  = - front + right + cycle; 
	chassis->target.motor_speed[WHEEL_RB]  = - front - right + cycle; 
	
	
}


static void Chassis_Pid_Calculate(Chassis_t* chassis)
{
	if(chassis->pid_mode == SPEED_MODE)
	{
	  for(uint8_t i = 0;i < 4;i++)
		{
			chassis->wheel[i]->ctrl->speed_ctrl->target = chassis->target.motor_speed[i];
			chassis->wheel[i]->ctrl->speed_ctrl->measure = chassis->wheel[i]->rx_info->encoder_speed;
			chassis->wheel[i]->ctrl->speed_ctrl->err = chassis->wheel[i]->ctrl->speed_ctrl->target - chassis->wheel[i]->ctrl->speed_ctrl->measure;
			
			single_pid_ctrl(chassis->wheel[i]->ctrl->speed_ctrl);
			
			chassis->out.wheel_initial_out[i] = chassis->wheel[i]->ctrl->speed_ctrl->out;
		}
	
	}
  else if(chassis->pid_mode == POSITION_MODE)
	{
		for(uint8_t i = 0;i < 4;i++)
		{
			chassis->wheel[i]->ctrl->angle_ctrl_outer->target = chassis->target.motor_position[i];
			chassis->wheel[i]->ctrl->angle_ctrl_outer->measure = chassis->wheel[i]->rx_info->encoder_sum;
			chassis->wheel[i]->ctrl->angle_ctrl_outer->err = chassis->wheel[i]->ctrl->angle_ctrl_outer->target - chassis->wheel[i]->ctrl->angle_ctrl_outer->measure;
			
			single_pid_ctrl(chassis->wheel[i]->ctrl->angle_ctrl_outer);
			
			chassis->wheel[i]->ctrl->angle_ctrl_inner->target = chassis->wheel[i]->ctrl->angle_ctrl_outer->out;
			chassis->wheel[i]->ctrl->angle_ctrl_inner->measure = chassis->wheel[i]->rx_info->encoder_speed;
			chassis->wheel[i]->ctrl->angle_ctrl_inner->err = chassis->wheel[i]->ctrl->angle_ctrl_inner->target - chassis->wheel[i]->ctrl->angle_ctrl_inner->measure;
			
			single_pid_ctrl(chassis->wheel[i]->ctrl->angle_ctrl_inner);
			
			chassis->out.wheel_initial_out[i] = chassis->wheel[i]->ctrl->angle_ctrl_inner->out;
		}
	}

}



