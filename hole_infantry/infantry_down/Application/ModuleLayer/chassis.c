#include "chassis.h"
#include "gimbal.h"
#include "infantry.h"

static void Chassis_Status_Update(Chassis_t* chassis);
static void Chassis_Target_Update(Chassis_t* chassis);
static void Chassis_Speed_Calculate(Chassis_t* chassis);
static void Chassis_Offline_Update(Chassis_t* chassis);
static void Chassis_Offline_Process(Chassis_t* chassis);
static void Chassis_Pid_Calculate(Chassis_t* chassis);
static void Chassis_Cmd_Transmit(Chassis_t* chassis);
static void Chassis_Work(Chassis_t* chassis);

Chassis_t  chassis = {
	.wheel = &wheel_group,
	
	.pid_mode = SPEED_MODE,
	
	.work = Chassis_Work, 
};


static void Chassis_Status_Update(Chassis_t* chassis)
{
	switch (infantry.mode)
	{
	  case I_SLEEP:
			chassis->mode = C_SLEEP;
			break;
		
		case I_INIT:
			chassis->mode = C_INIT;
			break;
		
		case I_MEC:
		case I_TURN:
		case I_HOLE:
			chassis->mode = C_BOSS;
			break;
		
		case I_IMU:
			chassis->mode = C_SLAVE;
			break;
		
		default:
			break;
	
	}
	
	if(infantry.flag.chassis_off == true)
	{
		chassis->mode = C_SLEEP;
	}
	
}


static void Chassis_Target_Update(Chassis_t* chassis)
{
	int16_t yaw_angle_err;
	float yaw_angle_err_rad = gimbal.info.yaw_mec_err;
	
	int16_t front_speed = rc_sensor.info->ch3/660.f * FRONT_SPEED_MAX;
  int16_t right_speed = rc_sensor.info->ch2/660.f * RIGHT_SPEED_MAX;
	int16_t cycle_speed = rc_sensor.info->ch0/660.f * CYCLE_SPEED_MAX;
	
	if (fabs(yaw_angle_err_rad) > PI/2) 
  {
    front_speed *= -1;
    right_speed *= -1;
  }
	
	switch (chassis->mode)
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
      
	    // front和right值计算
	    chassis->target.front_speed = front_speed * cos(yaw_angle_err_rad) - right_speed * sin(yaw_angle_err_rad);
	    chassis->target.right_speed = right_speed * cos(yaw_angle_err_rad) + front_speed * sin(yaw_angle_err_rad);
	
	    chassis->target.cycle_speed = yaw_angle_err_rad * yaw_angle_err_rad*sgn(yaw_angle_err_rad) /PI*30;
			
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


static void Chassis_Offline_Update(Chassis_t* chassis)
{
	static uint8_t offline_id[WHEEL_CNT] = {0,0,0,0};
  uint8_t offline_cnt = 0;
	
	for(uint8_t i = 0;i<WHEEL_CNT;i++)
	{
		if(chassis->wheel->motor[i]->state == DEV_OFFLINE)
		{
			offline_id[i] = 1;
		}
		else{
		  offline_id[i] = 0;
		}
		
		offline_cnt += offline_id[i];
	}
	
  if(offline_cnt == 4)
	{
		infantry.flag.chassis_off = true;
	}
	else{
	  infantry.flag.chassis_off = false;
	}

}

static void Chassis_Offline_Process(Chassis_t* chassis)
{
	for(uint8_t i = 0;i< WHEEL_CNT;i++)
	{
		chassis->out.wheel_end_out[i] = 0;
	}
	
}

static void Chassis_Pid_Calculate(Chassis_t* chassis)
{
	if(chassis->mode == C_SLEEP)
	{
		Chassis_Offline_Process(chassis);
	}
	else if(chassis->pid_mode == SPEED_MODE)
	{
	  for(uint8_t i = 0;i < 4;i++)
		{
			chassis->wheel->motor[1]->ctrl->speed_ctrl->target = chassis->target.motor_speed[i];
			chassis->wheel->motor[i]->ctrl->speed_ctrl->measure = chassis->wheel->motor[i]->rx_info->encoder_speed;
			chassis->wheel->motor[i]->ctrl->speed_ctrl->err = chassis->wheel->motor[i]->ctrl->speed_ctrl->target - chassis->wheel->motor[i]->ctrl->speed_ctrl->measure;
			
			single_pid_ctrl(chassis->wheel->motor[i]->ctrl->speed_ctrl);
			
			chassis->out.wheel_initial_out[i] = chassis->wheel->motor[i]->ctrl->speed_ctrl->out;
		}
	
	}
  else if(chassis->pid_mode == POSITION_MODE)
	{
		for(uint8_t i = 0;i < 4;i++)
		{
			chassis->wheel->motor[i]->ctrl->angle_ctrl_outer->target = chassis->target.motor_position[i];
			chassis->wheel->motor[i]->ctrl->angle_ctrl_outer->measure = chassis->wheel->motor[i]->rx_info->encoder_sum;
			chassis->wheel->motor[i]->ctrl->angle_ctrl_outer->err = chassis->wheel->motor[i]->ctrl->angle_ctrl_outer->target - chassis->wheel->motor[i]->ctrl->angle_ctrl_outer->measure;
			
			single_pid_ctrl(chassis->wheel->motor[i]->ctrl->angle_ctrl_outer);
			
			chassis->wheel->motor[i]->ctrl->angle_ctrl_inner->target = chassis->wheel->motor[i]->ctrl->angle_ctrl_outer->out;
			chassis->wheel->motor[i]->ctrl->angle_ctrl_inner->measure = chassis->wheel->motor[i]->rx_info->encoder_speed;
			chassis->wheel->motor[i]->ctrl->angle_ctrl_inner->err = chassis->wheel->motor[i]->ctrl->angle_ctrl_inner->target - chassis->wheel->motor[i]->ctrl->angle_ctrl_inner->measure;
			
			single_pid_ctrl(chassis->wheel->motor[i]->ctrl->angle_ctrl_inner);
			
			chassis->out.wheel_initial_out[i] = chassis->wheel->motor[i]->ctrl->angle_ctrl_inner->out;
		}
	}

}


static void Chassis_Cmd_Transmit(Chassis_t* chassis)
{
	chassis->wheel->motor[WHEEL_RF]->tx_info->torque = chassis->out.wheel_end_out[WHEEL_RF];
	chassis->wheel->motor[WHEEL_RB]->tx_info->torque = chassis->out.wheel_end_out[WHEEL_RB];
	chassis->wheel->motor[WHEEL_LF]->tx_info->torque = chassis->out.wheel_end_out[WHEEL_LF];
	chassis->wheel->motor[WHEEL_LB]->tx_info->torque = chassis->out.wheel_end_out[WHEEL_LB];
	
	chassis->wheel->group_set_torque(chassis->wheel);
	
}


static void Chassis_Work(Chassis_t* chassis)
{
	Chassis_Offline_Update(chassis);
  Chassis_Status_Update(chassis);
	Chassis_Target_Update(chassis);
	Chassis_Speed_Calculate(chassis);
	Chassis_Pid_Calculate(chassis);
	Chassis_Cmd_Transmit(chassis);
	
}

