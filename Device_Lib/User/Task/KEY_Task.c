/* Includes ------------------------------------------------------------------*/
#include "KEY_Task.h"

#define DEBOUNCE_TIME 60  // 消抖时间
#define KEY_RELEASED   GPIO_PIN_SET
#define KEY_PRESSED    GPIO_PIN_RESET

// 按键引脚定义
#define KEY1_PIN       GPIO_PIN_5
#define KEY2_PIN       GPIO_PIN_6
#define KEY3_PIN       GPIO_PIN_7
#define KEY_PORT       GPIOA

// 全局计数变量
uint8_t key1_num = 0;
uint8_t key2_num = 0;
uint8_t key3_num = 0;
uint8_t led1_cnt = 0;
uint8_t led2_cnt = 0;
uint8_t led3_cnt = 0;

/* 新增按键状态结构体 */
typedef struct {
  uint8_t last_state;      // 上次按键状态
  uint32_t last_time;      // 上次触发时间
} KeyState;

KeyState key1 = {KEY_RELEASED, 0};
KeyState key2 = {KEY_RELEASED, 0};
KeyState key3 = {KEY_RELEASED, 0};

/* Exported functions --------------------------------------------------------*/
void Key_Scan_Task(void)
{
  uint32_t current_time = HAL_GetTick();
  
  if (HAL_GPIO_ReadPin(KEY_PORT, KEY1_PIN) == KEY_PRESSED)
	{
    if ((key1.last_state == KEY_RELEASED) && 
        ((current_time - key1.last_time) > DEBOUNCE_TIME)) 
    {
      key1_num++;
      led1_cnt++;
      key1.last_state = KEY_PRESSED;
      key1.last_time = current_time;
    }
  }
	else
	{
    key1.last_state = KEY_RELEASED;
  }

  /* 按键2扫描 */
  if (HAL_GPIO_ReadPin(KEY_PORT, KEY2_PIN) == KEY_PRESSED)
	{
    if ((key2.last_state == KEY_RELEASED) && 
        ((current_time - key2.last_time) > DEBOUNCE_TIME)) 
    {
      key2_num++;
      led2_cnt++;
      key2.last_state = KEY_PRESSED;
      key2.last_time = current_time;
    }
  }
	else
	{
    key2.last_state = KEY_RELEASED;
  }

  /* 按键3扫描 */
  if (HAL_GPIO_ReadPin(KEY_PORT, KEY3_PIN) == KEY_PRESSED)
	{
    if ((key3.last_state == KEY_RELEASED) && 
        ((current_time - key3.last_time) > DEBOUNCE_TIME)) 
    {
      key3_num++;
      led3_cnt++;
      key3.last_state = KEY_PRESSED;
      key3.last_time = current_time;
    }
  }
	else
	{
    key3.last_state = KEY_RELEASED;
  }
}
