/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */
/**DMA Memory Selection**/
  #define MEMORY0 0
  #define MEMORY1 1
  #define MEMORYRESET 2
	
	/** DMA Data Length**/	
	#define RC_FRAME_LEN        18U         //Length of received data per frame
	#define RC_FRAME_LEN_BACK   7U          //Extra length for stability
	#define RC_CH_MAX_RELATIVE 660.0f       

	/** RC Channel Config **/
	#define RC_CH_VALUE_MIN ((uint16_t)364 )
	#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
	#define RC_CH_VALUE_MAX ((uint16_t)1684)
	
	#define RC_CH0    ((uint8_t)0)
	#define RC_CH1    ((uint8_t)1)
	#define RC_CH2    ((uint8_t)2)
	#define RC_CH3    ((uint8_t)3)
	#define RC_CH4    ((uint8_t)4)


	/** RC Switch Config**/	
	#define RC_SW_Right ((uint8_t)0)
	#define RC_SW_Left  ((uint8_t)1)
	
	#define RC_SW_UP    ((uint16_t)1)	
	#define RC_SW_DOWN  ((uint16_t)2)
	#define RC_SW_MID   ((uint16_t)3)
	
	
	/** Mouse Config**/
	#define MOUSE_X                 ((uint8_t)0)
	#define MOUSE_Y                 ((uint8_t)1)
	#define MOUSE_Z                 ((uint8_t)2)
		
	#define MOUSE_LEFT              ((uint8_t)3)
	#define MOUSE_RIGHT             ((uint8_t)4)
	
	#define MOUSE_PRESSED_OFFSET    ((uint8_t)0)
	#define MOUSE_SPEED_OFFSET      ((uint16_t)0)
	
	
	/** Keyboard Config **/
	#define KEY_W         ((uint8_t)0) 
	#define KEY_S         ((uint8_t)1)
	#define KEY_A         ((uint8_t)2)
	#define KEY_D         ((uint8_t)3)
	#define KEY_SHIFT     ((uint8_t)4)
	#define KEY_CTRL      ((uint8_t)5)
	#define KEY_Q         ((uint8_t)6)
	#define KEY_E         ((uint8_t)7)
	#define KEY_R         ((uint8_t)8)
	#define KEY_F         ((uint8_t)9)
	#define KEY_G         ((uint8_t)10)
	#define KEY_Z         ((uint8_t)11)
	#define KEY_X         ((uint8_t)12)
	#define KEY_C         ((uint8_t)13)
	#define KEY_V         ((uint8_t)14)
	#define KEY_B         ((uint8_t)15)
	#define KEY_OFFSET    ((uint8_t)0)
	
	#define KEY_PRESSED_OFFSET_W ((uint16_t)0x01<<0)
	#define KEY_PRESSED_OFFSET_S ((uint16_t)0x01<<1)
	#define KEY_PRESSED_OFFSET_A ((uint16_t)0x01<<2)
	#define KEY_PRESSED_OFFSET_D ((uint16_t)0x01<<3)
	#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)0x01<<4)
	#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)0x01<<5)
	#define KEY_PRESSED_OFFSET_Q ((uint16_t)0x01<<6)
	#define KEY_PRESSED_OFFSET_E ((uint16_t)0x01<<7)
	#define KEY_PRESSED_OFFSET_R ((uint16_t)0x01<<8)
	#define KEY_PRESSED_OFFSET_F ((uint16_t)0x01<<9)
	#define KEY_PRESSED_OFFSET_G ((uint16_t)0x01<<10)
	#define KEY_PRESSED_OFFSET_Z ((uint16_t)0x01<<11)
	#define KEY_PRESSED_OFFSET_X ((uint16_t)0x01<<12)
	#define KEY_PRESSED_OFFSET_C ((uint16_t)0x01<<13)
	#define KEY_PRESSED_OFFSET_V ((uint16_t)0x01<<14)
	#define KEY_PRESSED_OFFSET_B ((uint16_t)0x01<<15)
/* USER CODE END Private defines */

void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

