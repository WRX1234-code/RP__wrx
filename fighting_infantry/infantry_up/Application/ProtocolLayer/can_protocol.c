#include "can_protocol.h"
#include "board_protocol.h"

/**
 *  @brief  CAN1 接收数据
 */
void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		case 0x201:
//			Fric_Up_Motor.rx(&Fric_Up_Motor,rxBuf);
		  Fric_R_Motor.rx(&Fric_R_Motor,rxBuf);
		  break;
		
		case 0x202:
			Fric_L_Motor.rx(&Fric_L_Motor,rxBuf);
		  break;
		
		case 0x203:
			
		  break;
		
		case 0x11:
			Pitch_Motor.rx(&Pitch_Motor,rxBuf);
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
		case 0xD1:
			C_Board_Rx1(rxBuf);
		  break;
		
		case 0xD2:
			C_Board_Rx2(rxBuf);
		  break;
		case 0xD3:
			C_Board_Rx3(rxBuf);
		  break;
		case 0xD4:
			C_Board_Rx4(rxBuf);
		  break;
		
		case 0xD5:
			C_Board_Rx5(rxBuf);
		  break;
		
		
		default:
			break;
	}
}
