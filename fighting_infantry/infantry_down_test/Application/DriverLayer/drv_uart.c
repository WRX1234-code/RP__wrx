/**
 ******************************************************************************
 * @file        drv_uart.c
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
 
/* Includes ------------------------------------------------------------------*/
#include "drv_uart.h"
#include "string.h"
#include "judge_protocol.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart10;

/* Private macro -------------------------------------------------------------*/
#define USART5_RX_DATA_FRAME_LEN	(18u)	// 串口2数据帧长度
#define USART5_RX_BUF_LEN			(USART5_RX_DATA_FRAME_LEN)	// 串口2接收缓冲区长度


//#define USART5_RX_BUF_LEN	  600	//200
#define BUFF_SIZE 512
uint8_t rx_buff[BUFF_SIZE];

/* Private function prototypes -----------------------------------------------*/
__WEAK void USART1_rxDataHandler(uint8_t *rxBuf);
//__WEAK void USART3_rxDataHandler(uint8_t *rxBuf);
__WEAK void USART3_rxDataHandler(uint8_t *rxBuf);
__WEAK void USART5_rxDataHandler(uint8_t *rxBuf);
__WEAK void USART6_rxDataHandler(uint8_t *rxBuf);
static HAL_StatusTypeDef DMA_Start(DMA_HandleTypeDef *hdma, \
                            uint32_t SrcAddress, \
                            uint32_t DstAddress, \
                            uint32_t DataLength);
static void dma_m0_rxcplt_callback(DMA_HandleTypeDef *hdma);
static void dma_m1_rxcplt_callback(DMA_HandleTypeDef *hdma);
static void uart_rx_idle_callback(UART_HandleTypeDef* huart);
static HAL_StatusTypeDef DMAEx_MultiBufferStart_NoIT(DMA_HandleTypeDef *hdma, \
                                                    uint32_t SrcAddress, \
                                                    uint32_t DstAddress, \
                                                    uint32_t SecondMemAddress, \
                                                    uint32_t DataLength);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__attribute__((section (".AXI_SRAM"))) uint8_t usart10_dma_rxbuf[USART1_RX_BUF_LEN];
//uint8_t usart3_dma_rxbuf[2][USART3_RX_BUF_LEN];
__attribute__((section (".AXI_SRAM"))) uint8_t usart5_dma_rxbuf[2][USART5_RX_BUF_LEN];

__attribute__((section (".AXI_SRAM"))) uint8_t usart7_dma_rxbuf[USART7_RX_BUF_LEN];
/* Exported variables --------------------------------------------------------*/


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
	if(huart->Instance == UART5)
	{
		if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET) 
		{ 
			__HAL_DMA_DISABLE(huart->hdmarx); 

			((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT; 

			__HAL_DMA_SET_COUNTER(huart->hdmarx,USART5_RX_DATA_FRAME_LEN*2); 

			if(Size == USART5_RX_DATA_FRAME_LEN) 
			{ 
			USART5_rxDataHandler(usart5_dma_rxbuf[0]); 
			} 
		}
		else
		{
			__HAL_DMA_DISABLE(huart->hdmarx); 

			((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT); 
	 
			__HAL_DMA_SET_COUNTER(huart->hdmarx,USART5_RX_DATA_FRAME_LEN*2); 

			if(Size == USART5_RX_DATA_FRAME_LEN) 
			{ 
			USART5_rxDataHandler(usart5_dma_rxbuf[1]); 
			} 		 
		}							   
	}
										  
	huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
	
	/* Enalbe IDLE interrupt */
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	
  /* Enable the DMA transfer for the receiver request */
  SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
	
	__HAL_DMA_ENABLE(huart->hdmarx);
}

static void USART_DMAEx_MultiBuffer_Init(UART_HandleTypeDef *huart, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength)
{
 huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

// huart->RxEventType = HAL_UART_RXEVENT_IDLE;

 huart->RxXferSize    = DataLength*2;
 
	/*使能串口DMA模式*/
 SET_BIT(huart->Instance->CR3,USART_CR3_DMAR);

	/*使能uart中断*/
 __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE); 
 
	/*关闭DMA传输，确保可以设置DMA起点和终点*/
do{
      __HAL_DMA_DISABLE(huart->hdmarx);
  }while(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR & DMA_SxCR_EN);

	/*配置DMA起点地址和终点地址*/
((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->PAR = (uint32_t)&huart->Instance->RDR;
((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M0AR = (uint32_t)DstAddress;
((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M1AR = (uint32_t)SecondMemAddress;

	/*配置DMA数据传输长度*/
((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->NDTR = DataLength;

SET_BIT(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR, DMA_SxCR_DBM);

	/*使能DMA*/
__HAL_DMA_ENABLE(huart->hdmarx);	
}

void USART5_Init(void)
{
	USART_DMAEx_MultiBuffer_Init(&huart5, (uint32_t*)&usart5_dma_rxbuf[0], \
							    (uint32_t*)&usart5_dma_rxbuf[1], USART5_RX_BUF_LEN); // 接收完毕后重启
}


/**
 *	@brief	USART1 Initialization
 */
void USART1_Init(void)
{
	__HAL_UART_ENABLE_IT(&huart10, UART_IT_IDLE);
		
	
	HAL_UART_Receive_DMA(&huart10, usart10_dma_rxbuf, USART1_RX_BUF_LEN);
	
}


/* Private functions ---------------------------------------------------------*/
/**
  * @brief   clear idle it flag after uart receive a frame data
  * @param   uart IRQHandler id
  * @usage   call in DRV_UART_IRQHandler() function
  */
static void uart_rx_idle_callback(UART_HandleTypeDef* huart)
{
	__HAL_UART_CLEAR_IDLEFLAG(huart);
	
	if (huart == &huart10)
	{
		/* clear DMA transfer complete flag */
		__HAL_DMA_DISABLE(huart->hdmarx);
		/* handle dbus data dbus_buf from DMA */
		USART1_rxDataHandler(usart10_dma_rxbuf);
		memset(usart10_dma_rxbuf, 0, USART1_RX_BUF_LEN);
		/* restart dma transmission */	  
		__HAL_DMA_ENABLE(huart->hdmarx);		
	}
}


static HAL_StatusTypeDef DMA_Start(DMA_HandleTypeDef *hdma, \
                            uint32_t SrcAddress, \
                            uint32_t DstAddress, \
                            uint32_t DataLength)
{
	HAL_StatusTypeDef status = HAL_OK;
	
	/* Process locked */
	__HAL_LOCK(hdma);
	if(HAL_DMA_STATE_READY == hdma->State)
	{
		/* Change DMA peripheral state */
		hdma->State = HAL_DMA_STATE_BUSY;

		/* Initialize the error code */
		hdma->ErrorCode = HAL_DMA_ERROR_NONE;

		/* Configure the source, destination address and the data length */
		/* Clear DBM bit */
		((DMA_Stream_TypeDef  *)hdma->Instance)->CR &= (uint32_t)(~DMA_SxCR_DBM);

		/* Configure DMA Stream data length */
		((DMA_Stream_TypeDef  *)hdma->Instance)->NDTR = DataLength;

		/* Memory to Peripheral */
		if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
		{
			/* Configure DMA Stream destination address */
			((DMA_Stream_TypeDef  *)hdma->Instance)->PAR = DstAddress;

			/* Configure DMA Stream source address */
			((DMA_Stream_TypeDef  *)hdma->Instance)->M0AR = SrcAddress;
		}
		/* Peripheral to Memory */
		else
		{
			/* Configure DMA Stream source address */
			((DMA_Stream_TypeDef  *)hdma->Instance)->PAR = SrcAddress;

			/* Configure DMA Stream destination address */
			((DMA_Stream_TypeDef  *)hdma->Instance)->M0AR = DstAddress;
		}

		/* Enable the Peripheral */
		__HAL_DMA_ENABLE(hdma);
	}
	else
	{
		/* Process unlocked */
		__HAL_UNLOCK(hdma);

		/* Return error status */
		status = HAL_BUSY;
	} 
	return status; 	
}
/**
  * @brief   callback this function when uart interrupt 
  * @param   uart IRQHandler id
  * @usage   call in uart handler function USARTx_IRQHandler()
  */
void DRV_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    // 判断是否为空闲中断
		if(__HAL_UART_GET_FLAG(&huart10, UART_FLAG_IDLE))
	{
		__HAL_DMA_DISABLE(huart->hdmarx);
		
		__HAL_UART_CLEAR_IDLEFLAG(&huart10);
		
		HAL_UART_Receive_DMA(&huart10, usart10_dma_rxbuf, USART1_RX_BUF_LEN);
		
		USART1_rxDataHandler(usart10_dma_rxbuf);
//		memset(usart1_dma_rxbuf, 0, USART1_RX_BUF_LEN);
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
}

void WL_UART_printf(UART_HandleTypeDef *huart,char *format, ...)
{
		static char String[100];
		va_list arg;
		va_start(arg, format);
		vsprintf(String, format, arg);
		va_end(arg);
		if (huart->gState == HAL_UART_STATE_READY) 
		{
			HAL_UART_Transmit_IT(huart, (uint8_t *)String, strlen(String));
		}
		
}

/* rxData Handler [Weak] functions -------------------------------------------*/
/**
 *	@brief	[__WEAK] 需要在Potocol Layer中实现具体的 USART1 处理协议
 */
__WEAK void USART1_rxDataHandler(uint8_t *rxBuf)
{	
}

/**
 *	@brief	[__WEAK] 需要在Potocol Layer中实现具体的 USART3 处理协议
 */
__WEAK void USART3_rxDataHandler(uint8_t *rxBuf)
{	
}

/**
 *	@brief	[__WEAK] 需要在Potocol Layer中实现具体的 USART5 处理协议
 */
__WEAK void USART5_rxDataHandler(uint8_t *rxBuf)
{	
}

/**
 *	@brief	[__WEAK] 需要在Potocol Layer中实现具体的 USART6 处理协议
 */
__WEAK void USART6_rxDataHandler(uint8_t *rxBuf)
{	
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart7.Instance)
	{
		HAL_UART_Receive_DMA(&huart7, usart7_dma_rxbuf, USART7_RX_BUF_LEN);
		
		USART1_rxDataHandler(usart7_dma_rxbuf);
	}
}
