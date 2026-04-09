#ifndef __LED_TASK_H
#define __LED_TASK_H

#include "cmsis_os.h"
#include "gpio.h"
#include "WS2812b.h"

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t effect;
    uint16_t counter;
} LED_State_t;

#endif
