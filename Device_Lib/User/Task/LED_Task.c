/* Includes ------------------------------------------------------------------*/
#include "LED_Task.h"
#include "KEY_Task.h"
#include "ws2812b.h"
#include "gpiob_0.h"

extern SPI_HandleTypeDef hspi2;
extern uint8_t led1_cnt;
extern uint8_t led2_cnt;
extern uint8_t led3_cnt;

//GPIOB0引脚计时变量
extern uint32_t last_tick;
extern uint32_t last_pulse;

void StartLEDTask(void const * argument)
{
    static uint8_t led_last_cnt[3] = {0, 0, 0};
    static uint16_t no_led_on_cnt = 0;
    
    static uint8_t breath_brightness = 0;			// 当前亮度值 (0-255)
    static int8_t breath_direction = 1;				// 1=渐亮, -1=渐暗
    static uint8_t breath_speed_counter = 0;	// 呼吸速度控制计数器
    const uint8_t BREATH_SPEED = 5;						// 亮度变化速度（值越大越慢）
    const uint8_t MIN_BRIGHTNESS = 100;				// 最小亮度（防止休眠）
		const uint8_t MAX_BRIGHTNESS = 255;

    WS2812B_Init(&hspi2);
    
    // 开机动画
    for (int blink = 0; blink < 3; blink++)
		{
        for (uint8_t i = 0; i < 3; i++)
				{
            WS2812B_SetColor(i, 0, 0, 255);  // 蓝色
            WS2812B_Update();
            osDelay(250);
        }
    }
		
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	last_tick = HAL_GetTick();//记录初始时间
	last_pulse = last_tick;	//记录低电平时间

    for (;;)
		{
        Key_Scan_Task();
        
        // 按键响应逻辑
        if (led1_cnt != led_last_cnt[0])
				{
            WS2812B_SetColor(0, 0, 0, 255);  // Key1 蓝色
            no_led_on_cnt = 0;
            breath_brightness = 150;
        } 
        else if (led2_cnt != led_last_cnt[1])
				{
            WS2812B_SetColor(1, 255, 0, 0);  // Key2 红色
            no_led_on_cnt = 0;
            breath_brightness = 150;
        } 
        else if (led3_cnt != led_last_cnt[2])
				{
            WS2812B_SetColor(2, 0, 255, 0);  // Key3 绿色
            no_led_on_cnt = 0;
            breath_brightness = 150;
        } 
        else
				{
            no_led_on_cnt++;
						
            if (no_led_on_cnt > 500)
						{
//                breath_speed_counter++;
//                
//                if (breath_speed_counter >= BREATH_SPEED)
//								{
//                    breath_speed_counter = 0;
//                    breath_brightness += breath_direction;
//                    
//                    // 方向反转检测
//                    if (breath_brightness >= MAX_BRIGHTNESS)
//										{
//                        breath_brightness = MAX_BRIGHTNESS;
//                        breath_direction = -1;  // 达到峰值后变暗
//                    } 
//                    else if (breath_brightness <= MIN_BRIGHTNESS)
//										{
//                        breath_brightness = MIN_BRIGHTNESS;
//                        breath_direction = 1;   // 达到谷底后变亮
//                    }
//                    
//                    for (int i = 0; i < 3; i++)
//                        WS2812B_SetColor(i, breath_brightness, breath_brightness, breath_brightness);
//                }
					
								WS2812B_SetColor(0, 255, 255, 255);
								WS2812B_SetColor(1, 255, 255, 255);
								WS2812B_SetColor(2, 255, 255, 255);
						}
        }
        
        // 更新LED状态
        led_last_cnt[0] = led1_cnt;
        led_last_cnt[1] = led2_cnt;
        led_last_cnt[2] = led3_cnt;
        
        WS2812B_Update();
		
		GPIO_State_Operate();
		
        osDelay(1);  // 保持1ms延时保证实时性
    }
}