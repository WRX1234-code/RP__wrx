#include "chassis.h"
#include "gimbal.h"
#include "infantry.h"
#include "board_protocol.h"
#include "judge.h"
#include "cap.h"

static void Chassis_Init(Chassis_t* chassis);
static void Chassis_Status_Update(Chassis_t* chassis);
static void Chassis_Target_Update(Chassis_t* chassis);
static void Chassis_Inverse_Calculate(Chassis_t* chassis);
static void Chassis_Positive_Calculate(Chassis_t* chassis);
static void Chassis_Offline_Update(Chassis_t* chassis);
static void Chassis_Offline_Process(Chassis_t* chassis);
static void Chassis_Pid_Calculate(Chassis_t* chassis);
static void Chassis_Power_Limit(Chassis_t * chassis);
static void New_Chassis_Power_Limit(Chassis_t *chassis);
static void Chassis_Cmd_Transmit(Chassis_t* chassis);
static void Chassis_Work(Chassis_t* chassis);

Chassis_t  chassis = {
	.wheel = &wheel_group,
	.pid_mode = SPEED_MODE,
	.slip = {
		.slip_flag = false,
		.is_allot = true,
	  .wheel_speed_max_difference = 9.f,
	  .slip_low_out = 0,
	},
	.init = Chassis_Init, 
};



static void Chassis_Init(Chassis_t* chassis)
{
	chassis->work = Chassis_Work;
}


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
			chassis->mode = C_BOSS;
			break;
		
		case I_HOLE:
		  if(infantry.flag.hole_flag == true && infantry.flag.chassis_reset.value == true)
			{
				chassis->mode = C_SLAVE;
			}
			else if(infantry.flag.hole_flag == true && infantry.flag.chassis_reset.value == false)
			{
				chassis->mode = C_BOSS;
			}
			else if(infantry.flag.hole_flag == false && board.rx_meg->gimbal_meg.is_reach == false)
			{
				chassis->mode = C_BOSS;
			}
			else if(infantry.flag.hole_flag == false && board.rx_meg->gimbal_meg.is_reach == true)
			{
				chassis->mode = C_SLAVE;
			}
			break;
			
		case I_IMU:
		case I_TURN:
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
	float yaw_angle_err_rad = gimbal.info.yaw_mec_err_act;
	
	float front_speed,left_speed,cycle_speed;
	
	float last_front_cnt,last_left_cnt,now_front_cnt,now_left_cnt;
	
	now_front_cnt = step_limit_filter(rc_sensor.info->W.cnt - rc_sensor.info->S.cnt, last_front_cnt, 5);
	now_left_cnt = step_limit_filter(rc_sensor.info->A.cnt - rc_sensor.info->D.cnt, last_left_cnt, 5);
 
	if(infantry.ctrl == RC_CTRL)
	{
		front_speed = rc_sensor.info->ch3/660.f * FRONT_MAX_SPEED;
    left_speed = -rc_sensor.info->ch2/660.f * LEFT_MAX_SPEED;
	  cycle_speed = -rc_sensor.info->ch0/660.f * CYCLE_MAX_SPEED;
	}
	else{
	  front_speed = (float)now_front_cnt/ KEY_W_CNT_MAX* FRONT_MAX_SPEED;
	  left_speed = -(float)now_left_cnt/ KEY_A_CNT_MAX* LEFT_MAX_SPEED;
	  cycle_speed = -rc_sensor.info->mouse_vy *0.0001;
		cycle_speed = constrain(cycle_speed,-CYCLE_MAX_SPEED,CYCLE_MAX_SPEED);
	}
	
	last_front_cnt = now_front_cnt;
	last_left_cnt = now_left_cnt;
	
	
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
	    chassis->target.cycle_speed = cycle_speed;
	
			break;
		
		case C_SLAVE:
      
	    // front和right值计算
	    chassis->target.front_speed = front_speed * cos(yaw_angle_err_rad) - left_speed * sin(yaw_angle_err_rad);
	    chassis->target.left_speed = left_speed * cos(yaw_angle_err_rad) + front_speed * sin(yaw_angle_err_rad);
		
      if(infantry.flag.turn_flag == true)
			{
	      chassis->target.cycle_speed = TURN_CYCLE_SPEED;
			}		
			else{
				chassis->target.cycle_speed = yaw_angle_err_rad * yaw_angle_err_rad*sgn(yaw_angle_err_rad) /PI*0.01;
			}
		
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
	
	float yaw_angle_err_rad = gimbal.info.yaw_mec_err_act;
	
	chassis->measure.front_speed = (- speed_lf - speed_lb + speed_rb + speed_rf)/4;
	chassis->measure.left_speed = (speed_lf - speed_lb - speed_rb + speed_rf)/4;
	chassis->measure.cycle_speed = (speed_lf + speed_lb + speed_rb + speed_rf)/4;
	

  board.tx_pkt->car_pkt.v_x = chassis->measure.front_speed * cos(yaw_angle_err_rad) 
            + chassis->measure.left_speed  * sin(yaw_angle_err_rad);

  board.tx_pkt->car_pkt.v_y  = chassis->measure.left_speed  * cos(yaw_angle_err_rad) 
            - chassis->measure.front_speed * sin(yaw_angle_err_rad);
	
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


/**
  * @Name    Calculate_Predicted_Power
  * @brief   将功率用电流和转速表达
  * @param   电流  转速
  * @result  功率
**/
static float Calculate_Predicted_Power(float i, float w) {
    // 系数
    const float k_1 = 3e-07;
    const float k_2 = 1.23e-07;
	const float c = 4.081;//常数

    // 计算多项式曲面值
    float result = k_1*w*w + k_2*i*i + c + 1.99688994e-6f*i*w;

    return result;  
}
/**
  * @Name    Calculate_Current_Out
  * @brief   解算出电流输出
  * @param   目标功率  转速  原始电流
  * @result  电流
**/
uint32_t error_test;
static float Calculate_Current_Out(float target_power,float w,int16_t raw_curret)
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


/**
  * @Name    New_Chassis_Power_Limit
  * @brief   给底盘电机输出进行功率限制赋值
  * @param   chassis
**/
float limit=60;
uint8_t buf[5];

float k_cap=0.013;
static void New_Chassis_Power_Limit(Chassis_t *chassis)
{
		if (judge.pkt->buffer_energy < 40)
		{
			Chassis_Power_Limit(chassis);
			return;
		}

		/*计算预测功率*/
		float limit_output_current[4];

		for(uint8_t i =0;i<WHEEL_CNT;i++)
    {
			limit_output_current[i] = chassis->out.wheel_initial_out[i];
		}
		
		float motor_speed[4];

		for(uint8_t i =0;i<WHEEL_CNT;i++)
    {
			motor_speed[i] = chassis->wheel->motor[i]->rx_info->speed;
		}
		
		
		float power_fit = 0;
		float temp_power[4];
		float RF_speed_abs=fabs(motor_speed[WHEEL_RF]);
		float RB_speed_abs=fabs(motor_speed[WHEEL_RB]);
		float LF_speed_abs=fabs(motor_speed[WHEEL_LF]);
		float LB_speed_abs=fabs(motor_speed[WHEEL_LB]);
		float target_front_speed=chassis->target.front_speed;
		float target_left_speed =chassis->target.left_speed ;
		float target_cycle_speed=chassis->target.cycle_speed;
		buf[0]=(fabs(fabs(RF_speed_abs+LF_speed_abs)-fabs(LB_speed_abs+RB_speed_abs))>=chassis->slip.wheel_speed_max_difference);
		buf[1]=(fabs(target_front_speed)>(CHASSIS_MAX_SPEED/4.f));
		buf[2]=(fabs(target_left_speed))<(CHASSIS_MAX_SPEED/4.f);
		buf[3]=fabs(target_cycle_speed)<(CHASSIS_MAX_SPEED/6.f);
			
		/*不动态分配功率*/
		chassis->slip.is_allot=1;
		//判断前轮打滑,打滑前轮卸力
		if(buf[0]&& buf[1]&& buf[2]&& buf[3])
		{
			/*不进行打滑处理*/
			chassis->slip.slip_flag=1;
		}
		else if(fabs(target_front_speed)<=CHASSIS_MAX_SPEED/6.f)
		{
			chassis->slip.slip_flag=0;
		}
		
			
		if(chassis->slip.slip_flag==1)
		{
			if(fabs(gimbal.info.yaw_mec_err_raw) <= PI/2)
			{
				limit_output_current[WHEEL_RF] = chassis->slip.slip_low_out;
			  limit_output_current[WHEEL_LF] = chassis->slip.slip_low_out;
			}
			else{
			  limit_output_current[WHEEL_RB] = chassis->slip.slip_low_out;
			  limit_output_current[WHEEL_LB] = chassis->slip.slip_low_out;
			
			}	
		}
		
		//只在前进时给后轮分配更多功率，如果不是只前进或者旋转分量太大就后驱
		if((fabs(target_left_speed)>(CHASSIS_MAX_SPEED/4.f)||(fabs(target_cycle_speed)>CHASSIS_MAX_SPEED/5.f)))
		{
			chassis->slip.is_allot=1;
		}
			
		for(uint8_t i = 0; i < 4; i++)
		{
			//分配功率
			if(fabs(gimbal.info.yaw_mec_err_raw) <= PI/2)
			{
				if(i==WHEEL_RB||i==WHEEL_LB)
			  {
				  temp_power[i] = (2-chassis->slip.is_allot)*Calculate_Predicted_Power(limit_output_current[i], motor_speed[i]);
		  	}
			  else
			  {
				  temp_power[i] = chassis->slip.is_allot*Calculate_Predicted_Power(limit_output_current[i], motor_speed[i]);
			  }
			
			}
			else{
			  if(i==WHEEL_RF||i==WHEEL_LF)
			  {
				  temp_power[i] = (2-chassis->slip.is_allot)*Calculate_Predicted_Power(limit_output_current[i], motor_speed[i]);
		  	}
			  else
			  {
				  temp_power[i] = chassis->slip.is_allot*Calculate_Predicted_Power(limit_output_current[i], motor_speed[i]);
			  }
			
			}
			if(temp_power[i] > 0)
			{
				power_fit += temp_power[i];
			}
		
		}
		/*计算最大输出功率*/
		float max_power = judge.pkt->chassis_power_limit*1.15;

		
//		if(cap_tx_info.bit_control.cap_switch == 1)//①开超电
//		{
//			if (cap.status->status == DEV_ONLINE)//②如果电容在线
//			{
//					if (cap.info->cap_Ucr > 13)
//				{
//					max_power += (cap.info->cap_Ucr - 13.f) *k_cap + 10;
//				}
//			}
//		}
		
		float power_rate = 1;
		if(power_fit == 0)
		{
			power_rate = 1;
		}
		else{
		  power_rate = max_power / power_fit;//折算率
		}
		
		/*计算输出电流*/
		//预测功率大于最大功率才限制
		if (power_fit > max_power)
		{
			//通过折算后的功率、电机现在的转速、pid算出的电流来得到折算后的电流
			
			chassis->out.wheel_powerd_out[WHEEL_RF] = Calculate_Current_Out(temp_power[WHEEL_RF] * power_rate, motor_speed[WHEEL_RF],limit_output_current[WHEEL_RF]);
			chassis->out.wheel_powerd_out[WHEEL_RB] = Calculate_Current_Out(temp_power[WHEEL_RB] * power_rate, motor_speed[WHEEL_RB],limit_output_current[WHEEL_RB]);
			chassis->out.wheel_powerd_out[WHEEL_LF] = Calculate_Current_Out(temp_power[WHEEL_LF] * power_rate, motor_speed[WHEEL_LF],limit_output_current[WHEEL_LF]);
			chassis->out.wheel_powerd_out[WHEEL_LB] = Calculate_Current_Out(temp_power[WHEEL_LB] * power_rate, motor_speed[WHEEL_LB],limit_output_current[WHEEL_LB]);
			
		}
		else{
			chassis->out.wheel_powerd_out[WHEEL_RF] = limit_output_current[WHEEL_RF];
			chassis->out.wheel_powerd_out[WHEEL_RB] = limit_output_current[WHEEL_RB];
			chassis->out.wheel_powerd_out[WHEEL_LF] = limit_output_current[WHEEL_LF];
		  chassis->out.wheel_powerd_out[WHEEL_LB] = limit_output_current[WHEEL_LB];
		}
	
}


/**
  * @brief   中值滤波 + 趋势预测的功率估算
  * @note    更适合缓冲能量跳变的情况
  */
float Power_Estimate_Advanced(void)
{
    uint16_t buffer = judge.pkt->buffer_energy;
    float power_limit = judge.pkt->chassis_power_limit;
    
    // 环形缓冲区保存最近 5 个采样（500ms）
    static uint16_t buffer_ring[5] = {0};
    static uint8_t idx = 0;
    static float power_out = 0;
    
    const float dt = 0.1f;
    
    // 存入新数据
    buffer_ring[idx] = buffer;
    idx = (idx + 1) % 5;
    
    // 计算中值（去噪）
    uint16_t sorted[5];
    memcpy(sorted, buffer_ring, sizeof(sorted));
    // 简单冒泡排序取中值
    for (uint8_t i = 0; i < 4; i++) {
        for (uint8_t j = 0; j < 4-i; j++) {
            if (sorted[j] > sorted[j+1]) {
                uint16_t tmp = sorted[j];
                sorted[j] = sorted[j+1];
                sorted[j+1] = tmp;
            }
        }
    }
    uint16_t buffer_median = sorted[2];  // 中值
    
    // 计算趋势（最近 200ms 的变化）
    uint8_t idx_now = (idx + 5 - 1) % 5;      // 最新
    uint8_t idx_old = (idx + 5 - 3) % 5;      // 200ms 前
    int16_t trend = (int16_t)buffer_ring[idx_now] - (int16_t)buffer_ring[idx_old];
    
    // 预测未来 100ms 的缓冲
    float predicted_delta = (float)trend / 2.0f;  // 200ms → 100ms
    
    // 反推功率
    float power_raw;
    if (buffer_median == 0) {
        power_raw = power_limit;
    } else if (predicted_delta < 0) {
        // 趋势减少，功率在上升
        power_raw = power_limit + fabsf(predicted_delta) / dt;
    } else if (predicted_delta > 0) {
        // 趋势增加，功率在下降
        power_raw = power_limit - 2.0f;
    } else {
        power_raw = power_limit + 2.0f;
    }
    
    // 输出滤波
    const float alpha = 0.25f;
    power_out = alpha * power_raw + (1.0f - alpha) * power_out;
    
    // 限幅
    if (power_out < 0) power_out = 0;
    if (power_out > power_limit * 2.5f) power_out = power_limit * 2.5f;
    
    return power_out;
}


/**
  * @Name    Caluculate_All_Predicted_Power
  * @brief   计算出所有的预测功率通过指针传出到全局变量，方便debug
  * @param   底盘结构体  电机的功率  电机总功率  预测总功率和裁判系统收到的功率的差值
**/
static void Caluculate_All_Predicted_Power(Chassis_t *chassis,float *each_power,float *power_all,float *power_error)
{
	int16_t limit_output_current[4];
	float motor_speed[4];
	float power_fit = 0;
	
	limit_output_current[WHEEL_RF] = chassis->out.wheel_end_out[WHEEL_RF];
	limit_output_current[WHEEL_RB] = chassis->out.wheel_end_out[WHEEL_RB];
	limit_output_current[WHEEL_LF] = chassis->out.wheel_end_out[WHEEL_LF];
	limit_output_current[WHEEL_LB] = chassis->out.wheel_end_out[WHEEL_LB];

	motor_speed[WHEEL_RF] = chassis->wheel->motor[WHEEL_RF]->rx_info->speed;
	motor_speed[WHEEL_RB] = chassis->wheel->motor[WHEEL_RB]->rx_info->speed;
	motor_speed[WHEEL_LF] = chassis->wheel->motor[WHEEL_LF]->rx_info->speed;
	motor_speed[WHEEL_LB] = chassis->wheel->motor[WHEEL_LB]->rx_info->speed;

	for(uint8_t i = 0; i < 4; i++)
	{
		each_power[i] = Calculate_Predicted_Power(limit_output_current[i], motor_speed[i]);
		if(each_power[i] > 0)
		{
			power_fit += each_power[i];
		}
	}
	*power_all = power_fit;
	*power_error = power_fit - Power_Estimate_Advanced();
}
	



static void Chassis_Cmd_Transmit(Chassis_t* chassis)
{
	#if POWER_LIMIT_SWITCH == 0
	  chassis->out.wheel_end_out[WHEEL_RF] = chassis->out.wheel_initial_out[WHEEL_RF];
	  chassis->out.wheel_end_out[WHEEL_RB] = chassis->out.wheel_initial_out[WHEEL_RB];
	  chassis->out.wheel_end_out[WHEEL_LF] = chassis->out.wheel_initial_out[WHEEL_LF];
	  chassis->out.wheel_end_out[WHEEL_LB] = chassis->out.wheel_initial_out[WHEEL_LB];
	#else
	  chassis->out.wheel_end_out[WHEEL_RF] = chassis->out.wheel_powerd_out[WHEEL_RF];
	  chassis->out.wheel_end_out[WHEEL_RB] = chassis->out.wheel_powerd_out[WHEEL_RB];
	  chassis->out.wheel_end_out[WHEEL_LF] = chassis->out.wheel_powerd_out[WHEEL_LF];
	  chassis->out.wheel_end_out[WHEEL_LB] = chassis->out.wheel_powerd_out[WHEEL_LB];
	
	#endif
	
	
	#if CHASSIS_SWITCH == 0
	  chassis->wheel->motor[WHEEL_RF]->tx_info->torque = 0;
	  chassis->wheel->motor[WHEEL_RB]->tx_info->torque = 0;
	  chassis->wheel->motor[WHEEL_LF]->tx_info->torque = 0;
	  chassis->wheel->motor[WHEEL_LB]->tx_info->torque = 0;
	
	#else
	  chassis->wheel->motor[WHEEL_RF]->tx_info->torque = chassis->out.wheel_end_out[WHEEL_RF];
	  chassis->wheel->motor[WHEEL_RB]->tx_info->torque = chassis->out.wheel_end_out[WHEEL_RB];
	  chassis->wheel->motor[WHEEL_LF]->tx_info->torque = chassis->out.wheel_end_out[WHEEL_LF];
	  chassis->wheel->motor[WHEEL_LB]->tx_info->torque = chassis->out.wheel_end_out[WHEEL_LB];
	#endif
	
	chassis->wheel->group_set_torque(chassis->wheel);
	
}

float each_power[4];
float power_all;
float power_error;
static void Chassis_Work(Chassis_t* chassis)
{
	Chassis_Offline_Update(chassis);
  Chassis_Status_Update(chassis);
	Chassis_Positive_Calculate(chassis);
	Chassis_Target_Update(chassis);
	Chassis_Inverse_Calculate(chassis);
	Chassis_Pid_Calculate(chassis);
//	Chassis_Power_Limit(chassis);
	New_Chassis_Power_Limit(chassis);
	Chassis_Cmd_Transmit(chassis);
	Caluculate_All_Predicted_Power(chassis,each_power,&power_all,&power_error);
	
}

