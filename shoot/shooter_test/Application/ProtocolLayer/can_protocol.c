#include "can_protocol.h"

/**
 *  @brief  CAN1 接收数据
 */
void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		case ID_FRIC_UP:
			RM_Group.motor[FRIC_UP]->rx(RM_Group.motor[FRIC_UP],rxBuf);
		  break;
			
		case ID_FRIC_R:
			RM_Group.motor[FRIC_R]->rx(RM_Group.motor[FRIC_R],rxBuf);
		  break;
		
		case ID_FRIC_L:
			RM_Group.motor[FRIC_L]->rx(RM_Group.motor[FRIC_L],rxBuf);
		  break;
		
		case ID_DIAL:
			RM_Group.motor[DIAL]->rx(RM_Group.motor[DIAL],rxBuf);
		  break;
		
		case ID_GIMB_P:
		  Pitch.rx(&Pitch,rxBuf);	
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
		
		default:
			break;
	}
}
