#include "can_protocol.h"
#include "Chassis.h"
#include "communicate.h"


/**
 *  @brief  CAN1 接收数据
 */
void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{

		case 0x250:           //0x250
		{
			memcpy(&communicate_chassis_target,rxBuf,sizeof(Communicate_Chassis_Target_t));
			break;
		}
		

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
			{
				chassis_wheel.wheel_lf->rx(chassis_wheel.wheel_lf,rxBuf);
			  break;
			}
			case 0x202:
			{
				chassis_wheel.wheel_rf->rx(chassis_wheel.wheel_rf,rxBuf);
			  break;
			}
		  case 0x203:
			{
				chassis_wheel.wheel_lb->rx(chassis_wheel.wheel_lb,rxBuf);
			  break;
			}
		  case 0x204:
      {
				chassis_wheel.wheel_rb->rx(chassis_wheel.wheel_rb,rxBuf);
			  break;
			}
			default:
			break;
	}
	
		
}
