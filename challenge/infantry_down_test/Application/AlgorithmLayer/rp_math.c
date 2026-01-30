/**
 * @file        rp_math.c
 * @author      RobotPilots
 * @Version     v1.1
 * @brief       RobotPilots Robots' Math Libaray.
 * @update
 *              v1.0(11-September-2020)
 *              v1.1(13-November-2021)
 *                  1.增加位操作函数
 */
 
/* Includes ------------------------------------------------------------------*/
#include "rp_math.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/


/**
 *	@brief	过半圈处理 angle：源数据 cycle:数据范围
 */
float half_cycle(float angle, float max)
{
	if (abs(angle) > (max / 2.f))
	{
		if (angle >= 0)
			angle += -max;
		else
			angle += max;
	}
	return angle;
}

/**
 * @brief 步进式限幅滤波函数
 * @param new_value 当前采样值
 * @param last_value 上一次的滤波输出值
 * @param max_step 最大步进值（死区阈值）
 * @return 本次滤波后的值
 */
float step_limit_filter(float new_value, float last_value, float max_step)
{
    float filtered_value;
    float difference = new_value - last_value;

    // 如果变化量超过最大步进值，则进行限幅步进处理
    if (fabsf(difference) > max_step) {
        filtered_value = last_value + sgn(max_step) * max_step;
    } else {
        // 变化量在允许范围内，直接采用新采样值
        filtered_value = new_value;
    }
    
    return filtered_value;
}



/**
  * @brief  低通滤波,k越小滤波越好 
  */
float Lowpass(float X_last, float X_new, float K)
{
	return (X_last + (X_new - X_last) * K);
}


int16_t RampInt(int16_t final, int16_t now, int16_t ramp)
{
	int32_t buffer = 0;
	
	buffer = final - now;
	if (buffer > 0)
	{
		if (buffer > ramp)
			now += ramp;
		else
			now += buffer;
	}
	else
	{
		if (buffer < -ramp)
			now += -ramp;
		else
			now += buffer;
	}

	return now;
}

/**
  * @brief  angleum获取下一个周期的值，方向为正
  */
float get_next_periodic_value(float init_value, float current_value, float period) 
{
    // 计算当前值相对于初始值已经经历了多少个完整的周期
    int32_t cycles_completed = (int32_t)((current_value - init_value) / period);
    
    // 计算下一个周期的值
    float next_value = init_value + (cycles_completed + 1) * period;
    
    return next_value;
}

/**
  * @brief  angle获取下一个周期的值，方向为正
  */
float get_next_periodic_circle_value(float min, float max,float init_value, float current, float period) 
{
   if (max <= min) return current;
    
    float range = max - min;
	int32_t cycles_completed = (int32_t)((current - init_value) / period);
    // 计算下一个周期的值
    float next_value = init_value + (cycles_completed + 1) * period;
    

    if (next_value > max) {
        next_value -= range ;
    } 
    else if (next_value < min) {
         next_value += range ;
    }
    return next_value;
}

float RampFloat(float final, float now, float ramp)
{
	float buffer = 0;
	
	buffer = final - now;
	if (buffer > 0)
	{
		if (buffer > ramp)
			now += ramp;
		else
			now += buffer;
	}
	else
	{
		if (buffer < -ramp)
			now += -ramp;
		else
			now += buffer;
	}

	return now;	
}

float DeathZoom(float input, float center, float death)
{
	if(abs(input - center) < death)
		return center;
	return input;
}

uint16_t float_to_uint16(float x, float x_min, float x_max, uint8_t bits)
{
  float span=x_max-x_min;
  float offset=x_min;
    
  return (uint16_t)((x-offset)*((float)((1<<bits)-1))/span);
}

float uint16_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
