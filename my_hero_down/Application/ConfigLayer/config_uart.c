
#include "config_uart.h"
#include "rc_sensor.h"


/**
  * @Name    USART1_rxDataHandler
  * @brief   �Ӿ����ݸ���
**/
void USART1_rxDataHandler(uint8_t *rxBuf)
{

}

/**
  * @Name    USART3_rxDataHandler
  * @brief   ң��������
**/
void USART3_rxDataHandler(uint8_t *rxBuf)
{
	// ����ң������
	rc_sensor.update(&rc_sensor, rxBuf);//����Э��
	rc_sensor.check(&rc_sensor);
}

/**
  * @Name    USART6_rxDataHandler
  * @brief    
**/

void USART6_rxDataHandler(uint8_t *rxBuf)
{
	
}
