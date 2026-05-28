#include "can_protocol.h"
#include "board_protocol.h"
#include "cap.h"
#include "motor.h"
/**
 *  @brief  CAN1 接收数据
 */
void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		case ID_WHEEL_RF:
			wheel_motor[WHEEL_RF].rx(&wheel_motor[WHEEL_RF],rxBuf);
		  break;
		
		case ID_WHEEL_RB:
			wheel_motor[WHEEL_RB].rx(&wheel_motor[WHEEL_RB],rxBuf);
		  break;
		
		case ID_WHEEL_LF:
			wheel_motor[WHEEL_LF].rx(&wheel_motor[WHEEL_LF],rxBuf);
		  break;
		
		case ID_WHEEL_LB:
			wheel_motor[WHEEL_LB].rx(&wheel_motor[WHEEL_LB],rxBuf);
		  break;
		
		case ID_SUPER_CAP_RX :
			cap.rx(&cap,rxBuf);
		  break;
		
		case ID_WIRELESS_CHARGE:
			memcpy(&wireless_rx_info,rxBuf,sizeof(wireless_rx_info_t));
		  wireless_rx_info.charging_power = int16_to_float(wireless_rx_info.charging_power, 32000, -32000, 150, 0);
			break;
		
	
		default:
			break;
		
	}
}
/**
 *  @brief  CAN2 接收数据
 */
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
	
	switch (canId)
	{
		case ID_MEG_01:
			Board_Rx_Meg_01(&board,rxBuf);
      break;
		
		case ID_MEG_02:
			Board_Rx_Meg_02(&board,rxBuf);
      break;
		
		
		default:
			break;
	}
}
