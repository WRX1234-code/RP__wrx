#ifndef __MYCAN_H
#define __MYCAN_H

void CAN_FilterInit(void);
void CAN_Init(void);
void CAN_Send_Std_Data(uint32_t ID,const uint8_t* data,uint8_t len);
	




#endif
