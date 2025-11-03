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
#include "cap.h"
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
		I1=(int16_t)chassis_wheel->chassis_output[0];
		I2=(int16_t)chassis_wheel->chassis_output[1];
		I3=(int16_t)chassis_wheel->chassis_output[2];
		I4=(int16_t)chassis_wheel->chassis_output[3];
		
//		I1=(int16_t)chassis_wheel->wheel_lf->tx_info->torque_current;
//	  I2=(int16_t)chassis_wheel->wheel_rf->tx_info->torque_current;
//	  I3=(int16_t)chassis_wheel->wheel_lb->tx_info->torque_current;
//	  I4=(int16_t)chassis_wheel->wheel_rb->tx_info->torque_current;
		
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
	
	chassis_wheel->chassis_output[0]=chassis_wheel->wheel_lf->ctrl->speed_ctrl->out;
	chassis_wheel->chassis_output[1]=chassis_wheel->wheel_rf->ctrl->speed_ctrl->out;
	chassis_wheel->chassis_output[2]=chassis_wheel->wheel_lb->ctrl->speed_ctrl->out;
	chassis_wheel->chassis_output[3]=chassis_wheel->wheel_rb->ctrl->speed_ctrl->out;
	
	
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

float each_power[4];
float power_all;
float power_error;
int16_t error_test = 0;

float Calculate_Predicted_Power(float i, float w) {
    // 系数
    const float k_1 = 3e-07;
    const float k_2 = 1.23e-07;
	const float c = 4.081;//常数

    // 计算多项式曲面值
    float result = k_1*w*w + k_2*i*i + c + 1.99688994e-6f*i*w;

    return result;  
}


float Calculate_Current_Out(float target_power,float w,int16_t raw_curret)
{
	if (target_power < 0)
	{
		return raw_curret;
	}
	

	// 系数
  const float k_1 = 3e-07;
  const float k_2 = 1.23e-07;
	const float c = 4.081;
	//已知转速解出功率相同时加速时和减速时的电流
	float t = sqrt((1.99688994e-6f * w) * (1.99688994e-6f * w) - 4 * k_2 * (k_1 * w * w + c - target_power));
	float i_1 = (-(1.99688994e-6f * w) + t) / (2 * k_2);
	float i_2 = (-(1.99688994e-6f * w) - t) / (2 * k_2);
	
	/*通过原始电流正负判断用算出来的正电流还是负电流*/
	if (raw_curret > 0)
	{
		//检测解是否正确
		if (abs(Calculate_Predicted_Power(i_1, w) - target_power) > 1)
		{
			error_test++;  
		}
		
		return i_1;
	}
	else if (raw_curret < 0)
	{
		if (abs(Calculate_Predicted_Power(i_2, w) - target_power) > 1)
		{
			error_test++;
		}

		return i_2;
	}
	else
	{
		return 0;
	}
}


float limit=60;
uint8_t buf[5];
#if HERO_TYPE==2
float k_cap=20;
#else
float k_cap=13;
#endif

void New_Chassis_Power_Limit(Chassis_t chassis_wheel)
{
	

		/*计算预测功率*/
		int16_t limit_output_current[4];

	for(uint8_t i = 0; i < 4; i++)
	{
		limit_output_current[i]=chassis_wheel.chassis_output[i];
	}
	
	int16_t motor_speed[4];

	motor_speed[0] =chassis_wheel.wheel_lf->rx_info->speed;
	motor_speed[1] =chassis_wheel.wheel_rf->rx_info->speed;
	motor_speed[2] =chassis_wheel.wheel_lb->rx_info->speed;
	motor_speed[3] =chassis_wheel.wheel_rb->rx_info->speed;

		float power_fit;
		float temp_power[4];
		for(uint8_t i = 0; i < 4; i++)
		{
			#if HERO_TYPE==2
			float LF_speed_abs=abs(chassis->chassisLF->info->speed);
			float RF_speed_abs=abs(chassis->chassisRF->info->speed);
			float LB_speed_abs=abs(chassis->chassisLB->info->speed);
			float RB_speed_abs=abs(chassis->chassisRB->info->speed);
			float target_front_speed=chassis->base_info.target_front_speed;
			float target_right_speed=chassis->base_info.target_right_speed;
			float target_cycle_speed=chassis->base_info.target_cycle_speed;
			buf[0]=(LF_speed_abs+LF_speed_abs-(LB_speed_abs+RB_speed_abs)>=chassis->slip.max_speed_difference);
				buf[1]=(abs(target_front_speed)>(CHASSIS_MAX_SPEED/4.f));
				buf[2]=(target_right_speed)<(CHASSIS_MAX_SPEED/4.f);
				buf[3]=abs(target_cycle_speed)<(CHASSIS_MAX_SPEED/6.f);
			
			//chassis->slip.k_dynamic=chassis->slip.k_power;
			/*不动态分配功率*/
			chassis->slip.k_dynamic=1;
			//判断前轮打滑,打滑前轮卸力
			if((((LF_speed_abs+LF_speed_abs)-(LB_speed_abs+RB_speed_abs))>=chassis->slip.max_speed_difference)&& \
				(abs(target_front_speed)>(CHASSIS_MAX_SPEED/4.f)&&abs(target_right_speed)<(CHASSIS_MAX_SPEED/4.f)&&abs(target_cycle_speed)<(CHASSIS_MAX_SPEED/6.f))\
				)
			{
				/*不进行打滑处理*/
				//chassis->slip.slip_flag=1;
			}
			if(abs(target_front_speed)<=CHASSIS_MAX_SPEED/6.f)
			{
				chassis->slip.slip_flag=0;
			}
			
			if(chassis->slip.slip_flag==1)
			{
				chassis->base_info.output_chassisLF=chassis->slip.slip_low_current;
				chassis->base_info.output_chassisRF=chassis->slip.slip_low_current;
				chassis->slip.k_dynamic=1;
			}
			//只在前进时给后轮分配更多功率，如果不是只前进或者旋转分量太大就后驱
			if(chassis->slip.k_power>=2||chassis->slip.k_power<1||(abs(target_right_speed)>(CHASSIS_MAX_SPEED/4.f)||(abs(chassis->base_info.target_cycle_speed>CHASSIS_MAX_SPEED/5.f))))
			{
				chassis->slip.k_dynamic=1;
			}
			//分配功率
			if(i==2||i==3)
			{
				temp_power[i] = (2-chassis->slip.k_dynamic)*Calculate_Predicted_Power(limit_output_current[i], motor_speed[i]);
			}
			else
			{
				temp_power[i] = chassis->slip.k_dynamic*Calculate_Predicted_Power(limit_output_current[i], motor_speed[i]);
			}
			if(temp_power[i] > 0)
			{
				power_fit += temp_power[i];
			}
			#else
			temp_power[i] = Calculate_Predicted_Power(limit_output_current[i], motor_speed[i]);
			if(temp_power[i] > 0)
			{
				power_fit += temp_power[i];
			}
			#endif
			
		}
		/*计算最大输出功率*/
		float max_power=80;
		
//		if(communicate.car_data0_rx_info->car_state.bit.is_on_cap)//①开超电
//		{
			if (cap.state == CAP_ONLINE)//②如果电容在线
			{
					if (cap.cap_U > 13)
				{
					max_power += (cap.cap_U - 13.f) *k_cap + 10;
				}
			}
//		}
		
		float power_rate = max_power / power_fit;//折算率
		/*计算输出电流*/
		//预测功率大于最大功率才限制
		if (power_fit > max_power)
		{
			//通过折算后的功率、电机现在的转速、pid算出的电流来得到折算后的电流
			chassis_wheel.chassis_output[0]=Calculate_Current_Out(temp_power[0] * power_rate, chassis_wheel.wheel_lf->rx_info->speed,chassis_wheel.chassis_output[0]);
			chassis_wheel.chassis_output[1]=Calculate_Current_Out(temp_power[1] * power_rate, chassis_wheel.wheel_rf->rx_info->speed,chassis_wheel.chassis_output[1]);
			chassis_wheel.chassis_output[2]=Calculate_Current_Out(temp_power[2] * power_rate, chassis_wheel.wheel_lb->rx_info->speed,chassis_wheel.chassis_output[2]);
			chassis_wheel.chassis_output[3]=Calculate_Current_Out(temp_power[3] * power_rate, chassis_wheel.wheel_rb->rx_info->speed,chassis_wheel.chassis_output[3]);
			
		}
	
}








