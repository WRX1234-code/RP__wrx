#include "ws2812b.h"
#include "string.h"

SPI_HandleTypeDef *hspi2_ws = NULL;
uint8_t spi_buffer[LED_NUM * 24 + 224];
SemaphoreHandle_t ws2812_sem = NULL;

void WS2812B_Init(SPI_HandleTypeDef *hspi) {
    hspi2_ws = hspi;
    
    // 创建信号量（如果尚未创建）
    if(ws2812_sem == NULL) {
        ws2812_sem = xSemaphoreCreateBinary();
        if(ws2812_sem != NULL) {
            xSemaphoreGive(ws2812_sem);
        }
    }
    
    // 初始化缓冲区（所有LED关闭）
    memset(spi_buffer, 0, sizeof(spi_buffer));
}

void WS2812B_SetColor(uint16_t led, uint8_t g, uint8_t r, uint8_t b) {
    if (led >= LED_NUM) return;
    
    // GRB格式：绿色(8位) | 红色(8位) | 蓝色(8位)
    uint32_t color = (g << 16) | (r << 8) | b;
    
    // 将24位颜色转换为SPI数据
    for (int i = 0; i < 24; i++) {
        // 1码: 0xF8 (11111000) -> 高电平5周期(0.625μs), 低电平3周期(0.375μs)
        // 0码: 0xC0 (11000000) -> 高电平2周期(0.25μs), 低电平6周期(0.75μs)
        spi_buffer[led * 24 + i] = (color & (1 << (23 - i))) ? 0xF8 : 0xC0;
    }
		
		
		
}

void WS2812B_Update(void) {
    if(ws2812_sem == NULL || hspi2_ws == NULL) return;  
        // 启动DMA传输
        HAL_SPI_Transmit_DMA(hspi2_ws, spi_buffer, sizeof(spi_buffer));
    
}

// DMA传输完成回调函数
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi == hspi2_ws && ws2812_sem != NULL) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(ws2812_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
