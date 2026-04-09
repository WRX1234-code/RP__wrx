#ifndef __WS2812B_H
#define __WS2812B_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g0xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"

// 定义LED数量
#define LED_NUM 3

// 函数声明
void WS2812B_Init(SPI_HandleTypeDef *hspi);
void WS2812B_SetColor(uint16_t led, uint8_t g, uint8_t r, uint8_t b);
void WS2812B_Update(void);

// 外部变量声明
extern SPI_HandleTypeDef *hspi2_ws;
extern uint8_t spi_buffer[LED_NUM * 24 + 224];
extern SemaphoreHandle_t ws2812_sem;

#ifdef __cplusplus
}
#endif

#endif /* __WS2812B_H */
