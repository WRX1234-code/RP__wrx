#include "config_uart.h"
#include "rc_sensor.h"
#include "vision_protocol.h"
#include "Board_protocol.h"


/**
  * @Name    USART1_rxDataHandler
  * @brief   视觉数据更新
**/
void USART1_rxDataHandler(uint8_t *rxBuf)
{
	Vision_Rx_Data(&vision_rx_frame,rxBuf);
	
}

/**
  * @Name    USART3_rxDataHandler
  * @brief   遥控器更新
**/
void USART3_rxDataHandler(uint8_t *rxBuf)
{
	// 更新遥控数据
//	rc_sensor.update(&rc_sensor, rxBuf);//解析协议
//	rc_sensor.check(&rc_sensor);
}

/**
  * @Name    USART6_rxDataHandler
  * @brief   板间通信
**/

void USART6_rxDataHandler(uint8_t *rxBuf)
{
	C_Board_Rx_Data(&C_Board_Rx_Info,rxBuf);
}
