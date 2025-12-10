#include "can_protocol.h"
#include "Chassis_Motor.h"
#include "gimbal_Motor.h"
#include "cap_protocol.h"
#include "judge.h"
#include "cap.h"

/**
 *  @brief  CAN1 接收数据
 */
void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		case 0x33:
		Sd_Group.motor[L_F_Sd_M]->rx(Sd_Group.motor[L_F_Sd_M], rxBuf);
		break;
		case 0x44:
		Sd_Group.motor[L_B_Sd_M]->rx(Sd_Group.motor[L_B_Sd_M], rxBuf);
		break;
		case 0x201:
		Wheel_Group.motor[L_WHEEL_M]->rx(Wheel_Group.motor[L_WHEEL_M], rxBuf);
		break;
		default:
			break;
	}
}

/**
 *  @brief  CAN2 接收数据
 */
void CAN2_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		case 0x11:
		Sd_Group.motor[R_F_Sd_M]->rx(Sd_Group.motor[R_F_Sd_M], rxBuf);
		break;
		case 0x22:
		Sd_Group.motor[R_B_Sd_M]->rx(Sd_Group.motor[R_B_Sd_M], rxBuf);
		break;
		
		case 0x201:
 		Wheel_Group.motor[R_WHEEL_M]->rx(Wheel_Group.motor[R_WHEEL_M], rxBuf);
		break;
		
		default:
			break;
	}
}


/**
 *  @brief  CAN3 接收数据
 */
void CAN3_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		default:
			break;
	}
}
