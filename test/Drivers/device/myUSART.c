#include "stm32f4xx.h"                  // Device header
#include "usart.h"
#include "led.h"

uint8_t USART_send_data[8];
uint8_t USART_receive_data[8];

void Usart_Send(uint8_t* send_data,uint16_t len)
{
	HAL_UART_Transmit_IT(&huart1,send_data,len);
}

void Usart_Receive(uint8_t* receive_data,uint16_t len)
{
	HAL_UART_Receive_IT(&huart1,receive_data,len);
}
	
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//led_state state;
	//led led_color;
	if(huart->Instance==USART1)
	{
		Usart_Send(USART_send_data,sizeof(USART_send_data));
		if(USART_receive_data[0]==0x00)
		{
			//led_color=led_blue;
			if(USART_receive_data[1]==0x00)
			{
				//state=down;
				LED_SetState(&led_red,down,NULL);
			}
			else 
			{
			//	state=on;
				LED_SetState(&led_red,on,NULL);
			}
		}
		else if(USART_receive_data[0]==0x01)
		{
			//led_color=led_green;
			if(USART_receive_data[1]==0x00)
			{
				//state=down;
				LED_SetState(&led_green,down,NULL);
			}
			else 
			{
				//state=on;
				LED_SetState(&led_green,on,NULL);
			}
		}
		Usart_Receive(USART_receive_data,sizeof(USART_receive_data));
		//LED_SetState(&led_color,state,NULL);
		
	}
	
}
