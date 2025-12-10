/**
 ******************************************************************************
 * @file        drv_uart.h
 * @author      RobotPilots@2020
 * @brief       UART Driver Package(Based on HAL).
 ******************************************************************************
 * @attention
 * 
 * Copyright 2020 RobotPilots
 * 
 * @Version     V1.0
 * @date        15-August-2020
 ******************************************************************************
 */
#ifndef __DRV_UART_H
#define __DRV_UART_H

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void DRV_UART_IRQHandler(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void USART1_Init(void);
void USART2_Init(void);
void USART3_Init(void);
void USART5_Init(void);
void USART6_Init(void);
void WL_UART_printf(UART_HandleTypeDef *huart,char *format, ...);

extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart10;
extern UART_HandleTypeDef huart1;

#define USART1_RX_BUF_LEN     400
#endif
