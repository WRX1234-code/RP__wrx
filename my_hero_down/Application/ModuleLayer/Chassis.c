#include "device.h"
#include "rp_math.h"
#include "driver.h"
#include "rm_motor.h"
#include "Chassis.h"
#include "can.h"
#include "pid.h"
#include "rc_protocol.h"
#include "motor.h"
#include "communicate.h"
//#include "car.h"
//#include "communicate_protocol.h"

Chassis_t chassis_wheel;

void Chassis_Wheel_Init(Chassis_t* chassis_wheel)
{
	chassis_wheel->wheel_lf=&rm_motor[chas_lf];
	chassis_wheel->wheel_rf=&rm_motor[chas_rf];
	chassis_wheel->wheel_lb=&rm_motor[chas_lb];
	chassis_wheel->wheel_rb=&rm_motor[chas_rb];
	
	chassis_wheel->w.kp=0.1f;
	chassis_wheel->w.ki=0.02f;
	chassis_wheel->w.kd=0.005f;
	chassis_wheel->w.integral_max=1000.0f;
	chassis_wheel->w.out_max=2000.0f;

}

void Chassis_Remote_Receive(Chassis_t* chassis_wheel)
{

	chassis_wheel->chassis_speed_x=(float)communicate_chassis_target.x_target_speed;
	chassis_wheel->chassis_speed_y=(float)communicate_chassis_target.y_target_speed;
	chassis_wheel->chassis_speed_w=(float)communicate_chassis_target.w_target_speed;
}

void Chassis_Speed_Calculate(Chassis_t* chassis_wheel)
{
	float kw=1.0f;
	chassis_wheel->chassis_speed_w*=kw;
	
	chassis_wheel->wheel_lf->ctrl->speed_ctrl->target=chassis_wheel->chassis_speed_x+chassis_wheel->chassis_speed_y+chassis_wheel->chassis_speed_w;
	chassis_wheel->wheel_rf->ctrl->speed_ctrl->target=-chassis_wheel->chassis_speed_x+chassis_wheel->chassis_speed_y+chassis_wheel->chassis_speed_w;
	chassis_wheel->wheel_lb->ctrl->speed_ctrl->target=chassis_wheel->chassis_speed_x-chassis_wheel->chassis_speed_y+chassis_wheel->chassis_speed_w;
	chassis_wheel->wheel_rb->ctrl->speed_ctrl->target=-chassis_wheel->chassis_speed_x-chassis_wheel->chassis_speed_y+chassis_wheel->chassis_speed_w;
}

void Chassis_Send(Chassis_t* chassis_wheel)
{
	chassis_wheel->wheel_lf->tx_info->torque_current=chassis_wheel->wheel_lf->ctrl->speed_ctrl->out;
	chassis_wheel->wheel_rf->tx_info->torque_current=chassis_wheel->wheel_rf->ctrl->speed_ctrl->out;
	chassis_wheel->wheel_lb->tx_info->torque_current=chassis_wheel->wheel_lb->ctrl->speed_ctrl->out;
	chassis_wheel->wheel_rb->tx_info->torque_current=chassis_wheel->wheel_rb->ctrl->speed_ctrl->out;
	
	uint8_t send_data[8];
	
	int16_t I1,I2,I3,I4;
  
	
	if(communicate_chassis_target.heart_state==0)
	{
		I1=(int16_t)chassis_wheel->wheel_lf->tx_info->torque_current;
	  I2=(int16_t)chassis_wheel->wheel_rf->tx_info->torque_current;
	  I3=(int16_t)chassis_wheel->wheel_lb->tx_info->torque_current;
	  I4=(int16_t)chassis_wheel->wheel_rb->tx_info->torque_current;
		
		send_data[0]=I1>>8;
	  send_data[1]=I1;
	  send_data[2]=I2>>8;
	  send_data[3]=I2;
	  send_data[4]=I3>>8;
	  send_data[5]=I3;
	  send_data[6]=I4>>8;
	  send_data[7]=I4;
		

		CAN2_SendData(0x200,send_data);
	}

	else if(communicate_chassis_target.heart_state==1)
	{
		chassis_wheel->wheel_lf->single_sleep(chassis_wheel->wheel_lf);
		chassis_wheel->wheel_rf->single_sleep(chassis_wheel->wheel_rf);
		chassis_wheel->wheel_lb->single_sleep(chassis_wheel->wheel_lb);
		chassis_wheel->wheel_rb->single_sleep(chassis_wheel->wheel_rb);
	}
	
}

void Chassis_PID_Speed_Calculate(Chassis_t* chassis_wheel)
{
	chassis_wheel->wheel_lf->ctrl->speed_ctrl->measure=chassis_wheel->wheel_lf->rx_info->encoder_speed;
	chassis_wheel->wheel_rf->ctrl->speed_ctrl->measure=chassis_wheel->wheel_rf->rx_info->encoder_speed;
	chassis_wheel->wheel_lb->ctrl->speed_ctrl->measure=chassis_wheel->wheel_lb->rx_info->encoder_speed;
	chassis_wheel->wheel_rb->ctrl->speed_ctrl->measure=chassis_wheel->wheel_rb->rx_info->encoder_speed;
	
	chassis_wheel->wheel_lf->ctrl->speed_ctrl->err=chassis_wheel->wheel_lf->ctrl->speed_ctrl->target-chassis_wheel->wheel_lf->ctrl->speed_ctrl->measure;
	chassis_wheel->wheel_rf->ctrl->speed_ctrl->err=chassis_wheel->wheel_rf->ctrl->speed_ctrl->target-chassis_wheel->wheel_rf->ctrl->speed_ctrl->measure;
	chassis_wheel->wheel_lb->ctrl->speed_ctrl->err=chassis_wheel->wheel_lb->ctrl->speed_ctrl->target-chassis_wheel->wheel_lb->ctrl->speed_ctrl->measure;
	chassis_wheel->wheel_rb->ctrl->speed_ctrl->err=chassis_wheel->wheel_rb->ctrl->speed_ctrl->target-chassis_wheel->wheel_rb->ctrl->speed_ctrl->measure;
	
	single_pid_ctrl(chassis_wheel->wheel_lf->ctrl->speed_ctrl);
	single_pid_ctrl(chassis_wheel->wheel_rf->ctrl->speed_ctrl);
	single_pid_ctrl(chassis_wheel->wheel_lb->ctrl->speed_ctrl);
	single_pid_ctrl(chassis_wheel->wheel_rb->ctrl->speed_ctrl);
	
	constrain(chassis_wheel->wheel_lf->ctrl->speed_ctrl->out,-chassis_wheel->wheel_lf->ctrl->speed_ctrl->out_max,chassis_wheel->wheel_lf->ctrl->speed_ctrl->out_max);  
	constrain(chassis_wheel->wheel_rf->ctrl->speed_ctrl->out,-chassis_wheel->wheel_rf->ctrl->speed_ctrl->out_max,chassis_wheel->wheel_rf->ctrl->speed_ctrl->out_max);  
	constrain(chassis_wheel->wheel_lb->ctrl->speed_ctrl->out,-chassis_wheel->wheel_lb->ctrl->speed_ctrl->out_max,chassis_wheel->wheel_lb->ctrl->speed_ctrl->out_max);  
	constrain(chassis_wheel->wheel_rb->ctrl->speed_ctrl->out,-chassis_wheel->wheel_rb->ctrl->speed_ctrl->out_max,chassis_wheel->wheel_rb->ctrl->speed_ctrl->out_max);  
	
}

//void Chassis_Heart_Beat(Chassis_t* chassis_wheel)
//{
//	chassis_wheel->wheel_lf->single_heart_beat(chassis_wheel->wheel_lf);
//	chassis_wheel->wheel_rf->single_heart_beat(chassis_wheel->wheel_rf);
//	chassis_wheel->wheel_lb->single_heart_beat(chassis_wheel->wheel_lb);
//	chassis_wheel->wheel_rb->single_heart_beat(chassis_wheel->wheel_rb);
//	
//}

//void Chassis_Drive(Chassis_t* chassis_wheel)
//{
////	if(chassis_wheel->wheel_lf->state->status==DEV_OFFLINE&&chassis_wheel->wheel_rf->state->status==DEV_OFFLINE&&chassis_wheel->wheel_lb->state->status==DEV_OFFLINE&&chassis_wheel->wheel_rb->state->status==DEV_OFFLINE)
////	{
////		chassis_wheel->wheel_lf->single_sleep(chassis_wheel->wheel_lf);
////		chassis_wheel->wheel_rf->single_sleep(chassis_wheel->wheel_rf);
////		chassis_wheel->wheel_lb->single_sleep(chassis_wheel->wheel_lb);
////		chassis_wheel->wheel_rb->single_sleep(chassis_wheel->wheel_rb);

////  }		
//	  Chassis_Remote_Receive(chassis_wheel);
//		Chassis_Speed_Calculate(chassis_wheel);
//		Chassis_PID_Speed_Calculate(chassis_wheel);
//		Chassis_Send(chassis_wheel);
//}







