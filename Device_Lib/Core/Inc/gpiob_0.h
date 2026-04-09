#ifndef __GPIOB_0_H
#define __GPIOB_0_H

#include "stm32g030xx.h"
#include "gpio.h"
#include <stdbool.h>

//定义拉低引脚间隔时间和标志位
#define LOW_LEVEL_PERIOD	20000	//触发低电平时间间隔 ms
#define PULSE_PERIOD		 10    // 低电平持续时间 ms

void GPIO_State_Operate(void);

#endif
