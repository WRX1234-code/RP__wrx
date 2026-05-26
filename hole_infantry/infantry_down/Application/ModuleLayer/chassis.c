#include "chassis.h"
#include "gimbal.h"
#include "infantry.h"
#include "board_protocol.h"
#include "judge.h"

static void Chassis_Status_Update(Chassis_t* chassis);
static void Chassis_Target_Update(Chassis_t* chassis);
static void Chassis_Inverse_Calculate(Chassis_t* chassis);
static void Chassis_Positive_Calculate(Chassis_t* chassis);
static void Chassis_Offline_Update(Chassis_t* chassis);
static void Chassis_Offline_Process(Chassis_t* chassis);
static void Chassis_Pid_Calculate(Chassis_t* chassis);
static void Chassis_Power_Limit(Chassis_t * chassis);
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
	float yaw_angle_err_rad = gimbal.info.yaw_mec_err;
	
	float front_speed = rc_sensor.info->ch3/660.f * FRONT_MAX_SPEED;
  float left_speed = -rc_sensor.info->ch2/660.f * LEFT_MAX_SPEED;
	float cycle_speed = -rc_sensor.info->ch0/660.f * CYCLE_MAX_SPEED;
	
	if (fabs(yaw_angle_err_rad) > PI/2) 
  {
    front_speed *= -1.f;
    left_speed *= -1.f;
  }
	
	switch (chassis->mode)
	{
	  case C_SLEEP:
		case C_INIT:
      chassis->target.front_speed = 0;
		  chassis->target.left_speed = 0;
	    chassis->target.cycle_speed = 0;
	
	    break;
		
		case C_BOSS:	
		
			chassis->target.front_speed = front_speed;
		  chassis->target.left_speed = left_speed;
			if(infantry.flag.turn_flag == false)
			{
	      chassis->target.cycle_speed = cycle_speed;
			}
			else{
			 chassis->target.cycle_speed = TURN_CYCLE_SPEED; // 小陀螺速度
			}
			break;
		
		case C_SLAVE:
      
	    // front和right值计算
	    chassis->target.front_speed = front_speed * cos(yaw_angle_err_rad) - left_speed * sin(yaw_angle_err_rad);
	    chassis->target.left_speed = left_speed * cos(yaw_angle_err_rad) + front_speed * sin(yaw_angle_err_rad);
	
	    chassis->target.cycle_speed = yaw_angle_err_rad * yaw_angle_err_rad*sgn(yaw_angle_err_rad) /PI*30;
			
			break;
		
		
		default:
			break;
	}

}


static void Chassis_Inverse_Calculate(Chassis_t* chassis)
{
	float front = chassis->target.front_speed;
	float left = chassis->target.left_speed;
	float cycle = chassis->target.cycle_speed;
	
	float speed_sum;
	float K;
	
	speed_sum = fabs(front) + fabs(left) + fabs(cycle);
	
	if(speed_sum > CHASSIS_MAX_SPEED)
	{
		K = (float)CHASSIS_MAX_SPEED / (float)speed_sum;
	}
	else 
	{
		K = 1.f;
	}

	front *= K;
	left *= K;
	cycle *= K;
	
	chassis->target.motor_speed[WHEEL_LF]  = - front + left + cycle; 
	chassis->target.motor_speed[WHEEL_LB]  = - front - left + cycle;
	chassis->target.motor_speed[WHEEL_RB]  =   front - left + cycle; 
	chassis->target.motor_speed[WHEEL_RF]  =   front + left + cycle; 
	
}


static void Chassis_Positive_Calculate(Chassis_t* chassis)
{
	float speed_rf = chassis->wheel->motor[WHEEL_RF]->rx_info->speed;
	float speed_rb = chassis->wheel->motor[WHEEL_RB]->rx_info->speed;
	float speed_lf = chassis->wheel->motor[WHEEL_LF]->rx_info->speed;
	float speed_lb = chassis->wheel->motor[WHEEL_LB]->rx_info->speed;
	
	chassis->measure.front_speed = (- speed_lf - speed_lb + speed_rb + speed_rf)/4;
	chassis->measure.left_speed = (speed_lf - speed_lb - speed_rb + speed_rf)/4;
	chassis->measure.cycle_speed = (speed_lf + speed_lb + speed_rb + speed_rf)/4;
	
	board.tx_pkt->car_pkt.v_x = chassis->measure.front_speed;
	board.tx_pkt->car_pkt.v_y = chassis->measure.left_speed;
	
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
			chassis->wheel->motor[i]->ctrl->speed_ctrl->measure = chassis->wheel->motor[i]->rx_info->speed;
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
			chassis->wheel->motor[i]->ctrl->angle_ctrl_outer->measure = chassis->wheel->motor[i]->rx_info->motor_angle_sum;
			chassis->wheel->motor[i]->ctrl->angle_ctrl_outer->err = chassis->wheel->motor[i]->ctrl->angle_ctrl_outer->target - chassis->wheel->motor[i]->ctrl->angle_ctrl_outer->measure;
			
			single_pid_ctrl(chassis->wheel->motor[i]->ctrl->angle_ctrl_outer);
			
			chassis->wheel->motor[i]->ctrl->angle_ctrl_inner->target = chassis->wheel->motor[i]->ctrl->angle_ctrl_outer->out;
			chassis->wheel->motor[i]->ctrl->angle_ctrl_inner->measure = chassis->wheel->motor[i]->rx_info->speed;
			chassis->wheel->motor[i]->ctrl->angle_ctrl_inner->err = chassis->wheel->motor[i]->ctrl->angle_ctrl_inner->target - chassis->wheel->motor[i]->ctrl->angle_ctrl_inner->measure;
			
			single_pid_ctrl(chassis->wheel->motor[i]->ctrl->angle_ctrl_inner);
			
			chassis->out.wheel_initial_out[i] = chassis->wheel->motor[i]->ctrl->angle_ctrl_inner->out;
		}
	}

}



/**
  * @name    Chassis_Power_Limit
  * @brief   底盘功率限制(经典祖传算法)
  * @param   底盘 
  * @retval
  * @author  HWX
  * @date    2022-11-06
**/
static void Chassis_Power_Limit(Chassis_t * chassis)
{
	#if POWER_LIMIT_SWITCH == 1
	#else
	#endif
	{
		float limit_output_current[4];
	
		float buffer = (float)judge.pkt->buffer_energy;
		float heat_rate;//输出电流缩放比例
		float Limit_k; //轮组速度和缩放比例
		float CHAS_LimitOutput;//缩放后轮组最大速度之和
		float CHAS_TotalOutput;//轮组电流之和
		
		//获取理想的底盘输出
		for(uint8_t i = 0;i<WHEEL_CNT;i++)
		{
			limit_output_current[i] = chassis->out.wheel_initial_out[i];
		}
		
		float OUT_MAX = 0;
	
		OUT_MAX = CHASSIS_MAX_SPEED * 4;//最大速度之和
		
		if(buffer > 60)
		{
			buffer = 60;//防止飞坡之后缓冲250J变为正增益系数
		}
		
		Limit_k = buffer / 60.f;  //最大为1，飞坡后底盘一直最大速度运行
		
		if(buffer < 25)
		{
			Limit_k = Limit_k * Limit_k ;//缓冲没多小就更慢一点
		}
		else
		{
			Limit_k = Limit_k;// 缓冲能量还有比较多就限制一点
		}
			
		if(buffer < 60)
		{
			CHAS_LimitOutput = Limit_k * OUT_MAX; //只要缓冲能量没满才限制
		}
		else 
		{
			CHAS_LimitOutput = OUT_MAX;    //缓冲能量满的就全速前进
		}
			
		CHAS_TotalOutput = fabs(limit_output_current[0]) + fabs(limit_output_current[1]) + fabs(limit_output_current[2]) + fabs(limit_output_current[3]) ;
		
		heat_rate = CHAS_LimitOutput / CHAS_TotalOutput;//电流缩放比例 = 利用现在剩余缓冲能量算出的速度和限制比例 * 轮组最大速度和 / 解算出的理想轮组速度和
		
	  if(CHAS_TotalOutput >= CHAS_LimitOutput)
	  {
			for(uint8_t i = 0 ; i < 4 ; i++) 
			{	
				limit_output_current[i] = (int16_t)(limit_output_current[i] * heat_rate);	
			}
		}
		/*重新赋值*/
		for(uint8_t i=0;i<WHEEL_CNT;i++)
		{
			chassis->out.wheel_powerd_out[i] = limit_output_current[1];
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
	Chassis_Positive_Calculate(chassis);
	Chassis_Target_Update(chassis);
	Chassis_Inverse_Calculate(chassis);
	Chassis_Pid_Calculate(chassis);
	Chassis_Cmd_Transmit(chassis);
	
}

