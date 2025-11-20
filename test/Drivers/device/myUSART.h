#ifndef __MYUSART_H
#define __MYUSART_H

extern uint8_t USART_send_data[8];
extern uint8_t USART_receive_data[8];

void Usart_Send(uint8_t* USART_send_data,uint16_t len);
void Usart_Receive(uint8_t* USART_receive_data,uint16_t len);


#endif


