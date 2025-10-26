#include "can_protocol.h"
#include "Communicate.h"
#include "Gimbal.h"
#include "Shooter.h"

/**
 *  @brief  CAN1 接收数据
 */
void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		case 0x207:
			shoot.dial.dial_config->rx(shoot.dial.dial_config,rxBuf);
		  break;

		case 0x142:
			gimbal_motor.gimbal_y_motor.y_motor->get_info(gimbal_motor.gimbal_y_motor.y_motor,rxBuf);
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
		
		
		case 0x201:
			RM_Group1.motor[0]->rx(RM_Group1.motor[0],rxBuf);
		 
		  break;
		case 0x202:
			RM_Group1.motor[1]->rx(RM_Group1.motor[1],rxBuf);
			
		  break;
		case 0x203:
			RM_Group1.motor[2]->rx(RM_Group1.motor[2],rxBuf);
			
		  break;
		case 0x204:
			RM_Group1.motor[3]->rx(RM_Group1.motor[3],rxBuf);
			
		  break;
		case 0x205:
			RM_Group2.motor[0]->rx(RM_Group2.motor[0],rxBuf);
			
		  break;
		
		case 0x206:
			gim_p_motor->rx(gim_p_motor,rxBuf);
		  break;
		case 0x207:
			RM_Group2.motor[2]->rx(RM_Group2.motor[2],rxBuf);
			
		  break;

		default:
			break;
	}
}
