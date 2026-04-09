#include "gpiob_0.h"

uint32_t last_tick = 0;
uint32_t last_pulse = 0;
bool pulse_active = false;			//是否处于低电平

void GPIO_State_Operate()
{
	 uint32_t current_tick = HAL_GetTick();
        
        // 检查20秒间隔
        if ((current_tick - last_pulse) >= LOW_LEVEL_PERIOD)  // 20秒
        {
            last_pulse = current_tick;
            
            // PB0 拉低
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
            
            // 记录低电平开始时间
            last_tick = current_tick;
            pulse_active = true;
        }
        
        // 脉冲处理
        if (pulse_active)
        {
            if ((current_tick - last_tick) >= PULSE_PERIOD)  // 10ms
            {
                pulse_active = false;
                // PB0 释放
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
            }
        }
}
