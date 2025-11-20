#include "stm32f4xx.h"                  // Device header
#include "can.h"
#include "led.h"

void CAN_FilterInit(void)
{
	CAN_FilterTypeDef CAN_FilterStructure;
	CAN_FilterStructure.FilterActivation=CAN_FILTER_ENABLE;
	CAN_FilterStructure.FilterBank=0;
	CAN_FilterStructure.FilterFIFOAssignment=CAN_FILTER_FIFO0;
	CAN_FilterStructure.FilterIdHigh=0x0000;
	CAN_FilterStructure.FilterIdLow=0x0000;
	CAN_FilterStructure.FilterMaskIdHigh=0x0000;
	CAN_FilterStructure.FilterMaskIdLow=0x0000;
	CAN_FilterStructure.FilterMode=CAN_FILTERMODE_IDMASK;
	CAN_FilterStructure.FilterScale=CAN_FILTERSCALE_32BIT;
	CAN_FilterStructure.SlaveStartFilterBank=14;
	HAL_CAN_ConfigFilter(&hcan1,&CAN_FilterStructure);
}

void CAN_Init(void)
{
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
	
}

void CAN_Send_Std_Data(uint32_t ID,const uint8_t* data,uint8_t len)
{
	CAN_TxHeaderTypeDef CAN_TxHeaderStructure;
	CAN_TxHeaderStructure.DLC=len;
	CAN_TxHeaderStructure.ExtId=0;
	CAN_TxHeaderStructure.IDE=CAN_ID_STD;
	CAN_TxHeaderStructure.RTR=CAN_RTR_DATA;
	CAN_TxHeaderStructure.StdId=ID;
	CAN_TxHeaderStructure.TransmitGlobalTime=DISABLE;
	
	uint32_t txMailbox;
	
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)==0){}
		
	if(HAL_CAN_AddTxMessage(&hcan1,&CAN_TxHeaderStructure,data,&txMailbox)==HAL_OK)
	{
		LED_SetState(&led_red,blink,100);
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	
	CAN_RxHeaderTypeDef rxHeader;
	
	uint8_t bufferdata[8];
	if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)>0)
	{
	  HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rxHeader,bufferdata);
	}
	/*
	if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rxHeader,bufferdata)==HAL_OK)
	 {
	  LED_SetState(&led_green,blink,100);
	 }
	*/
	
	
	
}





