/* Includes ------------------------------------------------------------------*/
#include "COM_Task.h"

extern uint8_t key1_num;
extern uint8_t key2_num;
extern uint8_t key3_num;
extern union
{
  Interactive_Frame_t Frame;
	uint8_t InteractiveFrameBuffer[17];
} Interactive_Frame;

// 左上是零点，和电脑画图工具统一
#define X100	1216
#define Y100	589
#define X50		1157
#define Y50		589
#define X_OK	959
#define Y_OK	685
#define XYES	905
#define YYES	578

custom_client_data_t custom_client_data;

/* Exported functions --------------------------------------------------------*/
void StartCOMTask(void const * argument)
{
    Interactive_Frame_Init();

    custom_client_data.reserved=0;
		
    for(;;)
		{
			if(key1_num != 0)
			{
				custom_client_data.key_value1=79;		//	按键 O
				custom_client_data.key_value2=0;
				custom_client_data.x_position=0;	
				custom_client_data.mouse_left=0;
				custom_client_data.y_position=0;
				custom_client_data.mouse_right=0;
				memcpy(Interactive_Frame.InteractiveFrameBuffer+JUDGE_DATA_OFFSET,&custom_client_data,sizeof(custom_client_data_t));
				Append_CRC_Check_Sum(Interactive_Frame.InteractiveFrameBuffer,Interactive_Frame.InteractiveFrameBuffer[1]);
				HAL_UART_Transmit(&huart2,Interactive_Frame.InteractiveFrameBuffer,(Interactive_Frame.InteractiveFrameBuffer[1] + 9),0xff);
				osDelay(30);
							
				custom_client_data.key_value1=0;
				custom_client_data.key_value2=0;
				custom_client_data.x_position=0;	
				custom_client_data.mouse_left=0;
				custom_client_data.y_position=0;
				custom_client_data.mouse_right=0;
				memcpy(Interactive_Frame.InteractiveFrameBuffer+JUDGE_DATA_OFFSET,&custom_client_data,sizeof(custom_client_data_t));
				Append_CRC_Check_Sum(Interactive_Frame.InteractiveFrameBuffer,Interactive_Frame.InteractiveFrameBuffer[1]);
				HAL_UART_Transmit(&huart2,Interactive_Frame.InteractiveFrameBuffer,(Interactive_Frame.InteractiveFrameBuffer[1] + 9),0xff);
				osDelay(30);

				//	+100
				custom_client_data.key_value1=0;
				custom_client_data.key_value2=0;
				custom_client_data.x_position=X100 ;	
				custom_client_data.mouse_left=0;
				custom_client_data.y_position=Y100;
				custom_client_data.mouse_right=0;
				memcpy(Interactive_Frame.InteractiveFrameBuffer+JUDGE_DATA_OFFSET,&custom_client_data,sizeof(custom_client_data_t));
				Append_CRC_Check_Sum(Interactive_Frame.InteractiveFrameBuffer,Interactive_Frame.InteractiveFrameBuffer[1]);
				HAL_UART_Transmit(&huart2,Interactive_Frame.InteractiveFrameBuffer,(Interactive_Frame.InteractiveFrameBuffer[1] + 9),0xff);
				osDelay(50);

				custom_client_data.key_value1=0;
				custom_client_data.key_value2=0;
				custom_client_data.x_position=X100;	
				custom_client_data.mouse_left=1;
				custom_client_data.y_position=Y100;
				custom_client_data.mouse_right=0;
				memcpy(Interactive_Frame.InteractiveFrameBuffer+JUDGE_DATA_OFFSET,&custom_client_data,sizeof(custom_client_data_t));
				Append_CRC_Check_Sum(Interactive_Frame.InteractiveFrameBuffer,Interactive_Frame.InteractiveFrameBuffer[1]);
				HAL_UART_Transmit(&huart2,Interactive_Frame.InteractiveFrameBuffer,(Interactive_Frame.InteractiveFrameBuffer[1] + 9),0xff);
				osDelay(50);

				custom_client_data.key_value1=0;
				custom_client_data.key_value2=0;
				custom_client_data.x_position=X100;	
				custom_client_data.mouse_left=0;
				custom_client_data.y_position=Y100;
				custom_client_data.mouse_right=0;
				memcpy(Interactive_Frame.InteractiveFrameBuffer+JUDGE_DATA_OFFSET,&custom_client_data,sizeof(custom_client_data_t));
				Append_CRC_Check_Sum(Interactive_Frame.InteractiveFrameBuffer,Interactive_Frame.InteractiveFrameBuffer[1]);//���ݶεĳ�����8
				HAL_UART_Transmit(&huart2,Interactive_Frame.InteractiveFrameBuffer,(Interactive_Frame.InteractiveFrameBuffer[1] + 9),0xff);
				osDelay(50);

				//	确认
				custom_client_data.key_value1=0;
				custom_client_data.key_value2=0;
				custom_client_data.x_position=X_OK;	
				custom_client_data.mouse_left=0;
				custom_client_data.y_position=Y_OK;
				custom_client_data.mouse_right=0;
				memcpy(Interactive_Frame.InteractiveFrameBuffer+JUDGE_DATA_OFFSET,&custom_client_data,sizeof(custom_client_data_t));
				Append_CRC_Check_Sum(Interactive_Frame.InteractiveFrameBuffer,Interactive_Frame.InteractiveFrameBuffer[1]);//���ݶεĳ�����8
				HAL_UART_Transmit(&huart2,Interactive_Frame.InteractiveFrameBuffer,(Interactive_Frame.InteractiveFrameBuffer[1] + 9),0xff);
				osDelay(50);

				custom_client_data.key_value1=0;
				custom_client_data.key_value2=0;
				custom_client_data.x_position=X_OK;	
				custom_client_data.mouse_left=1;
				custom_client_data.y_position=Y_OK;
				custom_client_data.mouse_right=0;
				memcpy(Interactive_Frame.InteractiveFrameBuffer+JUDGE_DATA_OFFSET,&custom_client_data,sizeof(custom_client_data_t));
				Append_CRC_Check_Sum(Interactive_Frame.InteractiveFrameBuffer,Interactive_Frame.InteractiveFrameBuffer[1]);//���ݶεĳ�����8
				HAL_UART_Transmit(&huart2,Interactive_Frame.InteractiveFrameBuffer,(Interactive_Frame.InteractiveFrameBuffer[1] + 9),0xff);
				osDelay(50);

				custom_client_data.key_value1=0;
				custom_client_data.key_value2=0;
				custom_client_data.x_position=X_OK;	
				custom_client_data.mouse_left=0;
				custom_client_data.y_position=Y_OK;
				custom_client_data.mouse_right=0;
				memcpy(Interactive_Frame.InteractiveFrameBuffer+JUDGE_DATA_OFFSET,&custom_client_data,sizeof(custom_client_data_t));
				Append_CRC_Check_Sum(Interactive_Frame.InteractiveFrameBuffer,Interactive_Frame.InteractiveFrameBuffer[1]);//���ݶεĳ�����8
				HAL_UART_Transmit(&huart2,Interactive_Frame.InteractiveFrameBuffer,(Interactive_Frame.InteractiveFrameBuffer[1] + 9),0xff);
				osDelay(100);
				
				// 是
				custom_client_data.key_value1=0;
				custom_client_data.key_value2=0;
				custom_client_data.x_position=XYES;	
				custom_client_data.mouse_left=0;
				custom_client_data.y_position=YYES;
				custom_client_data.mouse_right=0;
				memcpy(Interactive_Frame.InteractiveFrameBuffer+JUDGE_DATA_OFFSET,&custom_client_data,sizeof(custom_client_data_t));
				Append_CRC_Check_Sum(Interactive_Frame.InteractiveFrameBuffer,Interactive_Frame.InteractiveFrameBuffer[1]);//���ݶεĳ�����8
				HAL_UART_Transmit(&huart2,Interactive_Frame.InteractiveFrameBuffer,(Interactive_Frame.InteractiveFrameBuffer[1] + 9),0xff);
				osDelay(50);

				custom_client_data.key_value1=0;
				custom_client_data.key_value2=0;
				custom_client_data.x_position=XYES;	
				custom_client_data.mouse_left=1;
				custom_client_data.y_position=YYES;
				custom_client_data.mouse_right=0;
				memcpy(Interactive_Frame.InteractiveFrameBuffer+JUDGE_DATA_OFFSET,&custom_client_data,sizeof(custom_client_data_t));
				Append_CRC_Check_Sum(Interactive_Frame.InteractiveFrameBuffer,Interactive_Frame.InteractiveFrameBuffer[1]);//���ݶεĳ�����8
				HAL_UART_Transmit(&huart2,Interactive_Frame.InteractiveFrameBuffer,(Interactive_Frame.InteractiveFrameBuffer[1] + 9),0xff);
				osDelay(50);

				custom_client_data.key_value1=0;
				custom_client_data.key_value2=0;
				custom_client_data.x_position=XYES;	
				custom_client_data.mouse_left=0;
				custom_client_data.y_position=YYES;
				custom_client_data.mouse_right=0;
				memcpy(Interactive_Frame.InteractiveFrameBuffer+JUDGE_DATA_OFFSET,&custom_client_data,sizeof(custom_client_data_t));
				Append_CRC_Check_Sum(Interactive_Frame.InteractiveFrameBuffer,Interactive_Frame.InteractiveFrameBuffer[1]);//���ݶεĳ�����8
				HAL_UART_Transmit(&huart2,Interactive_Frame.InteractiveFrameBuffer,(Interactive_Frame.InteractiveFrameBuffer[1] + 9),0xff);
				osDelay(50);
						
				custom_client_data.key_value1=79;
				custom_client_data.key_value2=0;
				custom_client_data.x_position=0;	
				custom_client_data.mouse_left=0;
				custom_client_data.y_position=0;
				custom_client_data.mouse_right=0;
				memcpy(Interactive_Frame.InteractiveFrameBuffer+JUDGE_DATA_OFFSET,&custom_client_data,sizeof(custom_client_data_t));
				Append_CRC_Check_Sum(Interactive_Frame.InteractiveFrameBuffer,Interactive_Frame.InteractiveFrameBuffer[1]);//���ݶεĳ�����8
				HAL_UART_Transmit(&huart2,Interactive_Frame.InteractiveFrameBuffer,(Interactive_Frame.InteractiveFrameBuffer[1] + 9),0xff);
				osDelay(30);
				
				custom_client_data.key_value1=0;
				custom_client_data.key_value2=0;
				custom_client_data.x_position=0;	
				custom_client_data.mouse_left=0;
				custom_client_data.y_position=0;
				custom_client_data.mouse_right=0;
				memcpy(Interactive_Frame.InteractiveFrameBuffer+JUDGE_DATA_OFFSET,&custom_client_data,sizeof(custom_client_data_t));
				Append_CRC_Check_Sum(Interactive_Frame.InteractiveFrameBuffer,Interactive_Frame.InteractiveFrameBuffer[1]);//���ݶεĳ�����8
				HAL_UART_Transmit(&huart2,Interactive_Frame.InteractiveFrameBuffer,(Interactive_Frame.InteractiveFrameBuffer[1] + 9),0xff);
				
				#if 1
				osDelay(30);
				
				// ------------------ 自定义 180转头 ------------------ //
				
				custom_client_data.key_value1=86;			//	按键	V
				custom_client_data.key_value2=0;
				custom_client_data.x_position=0;	
				custom_client_data.mouse_left=0;
				custom_client_data.y_position=0;
				custom_client_data.mouse_right=0;
				memcpy(Interactive_Frame.InteractiveFrameBuffer+JUDGE_DATA_OFFSET,&custom_client_data,sizeof(custom_client_data_t));
				Append_CRC_Check_Sum(Interactive_Frame.InteractiveFrameBuffer,Interactive_Frame.InteractiveFrameBuffer[1]);//���ݶεĳ�����8
				HAL_UART_Transmit(&huart2,Interactive_Frame.InteractiveFrameBuffer,(Interactive_Frame.InteractiveFrameBuffer[1] + 9),0xff);
				osDelay(30);
				
				custom_client_data.key_value1=0;
				custom_client_data.key_value2=0;
				custom_client_data.x_position=0;	
				custom_client_data.mouse_left=0;
				custom_client_data.y_position=0;
				custom_client_data.mouse_right=0;
				memcpy(Interactive_Frame.InteractiveFrameBuffer+JUDGE_DATA_OFFSET,&custom_client_data,sizeof(custom_client_data_t));
				Append_CRC_Check_Sum(Interactive_Frame.InteractiveFrameBuffer,Interactive_Frame.InteractiveFrameBuffer[1]);//���ݶεĳ�����8
				HAL_UART_Transmit(&huart2,Interactive_Frame.InteractiveFrameBuffer,(Interactive_Frame.InteractiveFrameBuffer[1] + 9),0xff);
				#endif
				
				#if 0
				osDelay(30);
				
				// ------------------ 自定义 180转头 ------------------ //
				
				custom_client_data.key_value1=82;			//	按键	R
				custom_client_data.key_value2=0;
				custom_client_data.x_position=0;	
				custom_client_data.mouse_left=0;
				custom_client_data.y_position=0;
				custom_client_data.mouse_right=0;
				memcpy(Interactive_Frame.InteractiveFrameBuffer+JUDGE_DATA_OFFSET,&custom_client_data,sizeof(custom_client_data_t));
				Append_CRC_Check_Sum(Interactive_Frame.InteractiveFrameBuffer,Interactive_Frame.InteractiveFrameBuffer[1]);//���ݶεĳ�����8
				HAL_UART_Transmit(&huart2,Interactive_Frame.InteractiveFrameBuffer,(Interactive_Frame.InteractiveFrameBuffer[1] + 9),0xff);
				osDelay(30);
				
				custom_client_data.key_value1=0;
				custom_client_data.key_value2=0;
				custom_client_data.x_position=0;	
				custom_client_data.mouse_left=0;
				custom_client_data.y_position=0;
				custom_client_data.mouse_right=0;
				memcpy(Interactive_Frame.InteractiveFrameBuffer+JUDGE_DATA_OFFSET,&custom_client_data,sizeof(custom_client_data_t));
				Append_CRC_Check_Sum(Interactive_Frame.InteractiveFrameBuffer,Interactive_Frame.InteractiveFrameBuffer[1]);//���ݶεĳ�����8
				HAL_UART_Transmit(&huart2,Interactive_Frame.InteractiveFrameBuffer,(Interactive_Frame.InteractiveFrameBuffer[1] + 9),0xff);
				#endif
				
				osDelay(150);

				key1_num --;
			}

			if(key2_num != 0)
			{
				custom_client_data.key_value1=72;			//	按键 H 
				custom_client_data.key_value2=0;
				custom_client_data.x_position=0;	
				custom_client_data.mouse_left=0;
				custom_client_data.y_position=0;
				custom_client_data.mouse_right=0;
				memcpy(Interactive_Frame.InteractiveFrameBuffer+JUDGE_DATA_OFFSET,&custom_client_data,sizeof(custom_client_data_t));
				Append_CRC_Check_Sum(Interactive_Frame.InteractiveFrameBuffer,Interactive_Frame.InteractiveFrameBuffer[1]);//���ݶεĳ�����8
				HAL_UART_Transmit(&huart2,Interactive_Frame.InteractiveFrameBuffer,(Interactive_Frame.InteractiveFrameBuffer[1] + 9),0xff);
				osDelay(30);

				custom_client_data.key_value1=0;
				custom_client_data.key_value2=0;
				custom_client_data.x_position=0;	
				custom_client_data.mouse_left=0;
				custom_client_data.y_position=0;
				custom_client_data.mouse_right=0;
				memcpy(Interactive_Frame.InteractiveFrameBuffer+JUDGE_DATA_OFFSET,&custom_client_data,sizeof(custom_client_data_t));
				Append_CRC_Check_Sum(Interactive_Frame.InteractiveFrameBuffer,Interactive_Frame.InteractiveFrameBuffer[1]);//���ݶεĳ�����8
				HAL_UART_Transmit(&huart2,Interactive_Frame.InteractiveFrameBuffer,(Interactive_Frame.InteractiveFrameBuffer[1] + 9),0xff);
				osDelay(30);

				custom_client_data.key_value1=17;			//	CTRL
				custom_client_data.key_value2=50;			//	2			远程买弹
				custom_client_data.x_position=0;	
				custom_client_data.mouse_left=0;
				custom_client_data.y_position=0;
				custom_client_data.mouse_right=0;
				memcpy(Interactive_Frame.InteractiveFrameBuffer+JUDGE_DATA_OFFSET,&custom_client_data,sizeof(custom_client_data_t));
				Append_CRC_Check_Sum(Interactive_Frame.InteractiveFrameBuffer,Interactive_Frame.InteractiveFrameBuffer[1]);//���ݶεĳ�����8
				HAL_UART_Transmit(&huart2,Interactive_Frame.InteractiveFrameBuffer,(Interactive_Frame.InteractiveFrameBuffer[1] + 9),0xff);
				osDelay(30);
				
				custom_client_data.key_value1=0;
				custom_client_data.key_value2=0;
				custom_client_data.x_position=0;	
				custom_client_data.mouse_left=0;
				custom_client_data.y_position=0;
				custom_client_data.mouse_right=0;
				memcpy(Interactive_Frame.InteractiveFrameBuffer+JUDGE_DATA_OFFSET,&custom_client_data,sizeof(custom_client_data_t));
				Append_CRC_Check_Sum(Interactive_Frame.InteractiveFrameBuffer,Interactive_Frame.InteractiveFrameBuffer[1]);//���ݶεĳ�����8
				HAL_UART_Transmit(&huart2,Interactive_Frame.InteractiveFrameBuffer,(Interactive_Frame.InteractiveFrameBuffer[1] + 9),0xff);
				osDelay(30);
				
				custom_client_data.key_value1=89;			//	按键	Y
				custom_client_data.key_value2=0;
				custom_client_data.x_position=0;	
				custom_client_data.mouse_left=0;
				custom_client_data.y_position=0;
				custom_client_data.mouse_right=0;
				memcpy(Interactive_Frame.InteractiveFrameBuffer+JUDGE_DATA_OFFSET,&custom_client_data,sizeof(custom_client_data_t));
				Append_CRC_Check_Sum(Interactive_Frame.InteractiveFrameBuffer,Interactive_Frame.InteractiveFrameBuffer[1]);//���ݶεĳ�����8
				HAL_UART_Transmit(&huart2,Interactive_Frame.InteractiveFrameBuffer,(Interactive_Frame.InteractiveFrameBuffer[1] + 9),0xff);
				osDelay(30);
				
				custom_client_data.key_value1=0;
				custom_client_data.key_value2=0;
				custom_client_data.x_position=0;	
				custom_client_data.mouse_left=0;
				custom_client_data.y_position=0;
				custom_client_data.mouse_right=0;
				memcpy(Interactive_Frame.InteractiveFrameBuffer+JUDGE_DATA_OFFSET,&custom_client_data,sizeof(custom_client_data_t));
				Append_CRC_Check_Sum(Interactive_Frame.InteractiveFrameBuffer,Interactive_Frame.InteractiveFrameBuffer[1]);//���ݶεĳ�����8
				HAL_UART_Transmit(&huart2,Interactive_Frame.InteractiveFrameBuffer,(Interactive_Frame.InteractiveFrameBuffer[1] + 9),0xff);
				osDelay(150);
				
				key2_num --;
			}
			
			if(key3_num != 0)
			{
				custom_client_data.key_value1=72;			//	按键 H 
				custom_client_data.key_value2=0;
				custom_client_data.x_position=0;	
				custom_client_data.mouse_left=0;
				custom_client_data.y_position=0;
				custom_client_data.mouse_right=0;
				memcpy(Interactive_Frame.InteractiveFrameBuffer+JUDGE_DATA_OFFSET,&custom_client_data,sizeof(custom_client_data_t));
				Append_CRC_Check_Sum(Interactive_Frame.InteractiveFrameBuffer,Interactive_Frame.InteractiveFrameBuffer[1]);//���ݶεĳ�����8
				HAL_UART_Transmit(&huart2,Interactive_Frame.InteractiveFrameBuffer,(Interactive_Frame.InteractiveFrameBuffer[1] + 9),0xff);
				osDelay(30);

				custom_client_data.key_value1=0;
				custom_client_data.key_value2=0;
				custom_client_data.x_position=0;	
				custom_client_data.mouse_left=0;
				custom_client_data.y_position=0;
				custom_client_data.mouse_right=0;
				memcpy(Interactive_Frame.InteractiveFrameBuffer+JUDGE_DATA_OFFSET,&custom_client_data,sizeof(custom_client_data_t));
				Append_CRC_Check_Sum(Interactive_Frame.InteractiveFrameBuffer,Interactive_Frame.InteractiveFrameBuffer[1]);//���ݶεĳ�����8
				HAL_UART_Transmit(&huart2,Interactive_Frame.InteractiveFrameBuffer,(Interactive_Frame.InteractiveFrameBuffer[1] + 9),0xff);
				osDelay(30);

				custom_client_data.key_value1=17;			//	CTRL
				custom_client_data.key_value2=49;			//	1			// 远程买血
				custom_client_data.x_position=0;	
				custom_client_data.mouse_left=0;
				custom_client_data.y_position=0;
				custom_client_data.mouse_right=0;
				memcpy(Interactive_Frame.InteractiveFrameBuffer+JUDGE_DATA_OFFSET,&custom_client_data,sizeof(custom_client_data_t));
				Append_CRC_Check_Sum(Interactive_Frame.InteractiveFrameBuffer,Interactive_Frame.InteractiveFrameBuffer[1]);//���ݶεĳ�����8
				HAL_UART_Transmit(&huart2,Interactive_Frame.InteractiveFrameBuffer,(Interactive_Frame.InteractiveFrameBuffer[1] + 9),0xff);
				osDelay(30);
				
				custom_client_data.key_value1=0;
				custom_client_data.key_value2=0;
				custom_client_data.x_position=0;	
				custom_client_data.mouse_left=0;
				custom_client_data.y_position=0;
				custom_client_data.mouse_right=0;
				memcpy(Interactive_Frame.InteractiveFrameBuffer+JUDGE_DATA_OFFSET,&custom_client_data,sizeof(custom_client_data_t));
				Append_CRC_Check_Sum(Interactive_Frame.InteractiveFrameBuffer,Interactive_Frame.InteractiveFrameBuffer[1]);//���ݶεĳ�����8
				HAL_UART_Transmit(&huart2,Interactive_Frame.InteractiveFrameBuffer,(Interactive_Frame.InteractiveFrameBuffer[1] + 9),0xff);
				osDelay(30);
				
				custom_client_data.key_value1=89;			//	按键	Y
				custom_client_data.key_value2=0;
				custom_client_data.x_position=0;	
				custom_client_data.mouse_left=0;
				custom_client_data.y_position=0;
				custom_client_data.mouse_right=0;
				memcpy(Interactive_Frame.InteractiveFrameBuffer+JUDGE_DATA_OFFSET,&custom_client_data,sizeof(custom_client_data_t));
				Append_CRC_Check_Sum(Interactive_Frame.InteractiveFrameBuffer,Interactive_Frame.InteractiveFrameBuffer[1]);//���ݶεĳ�����8
				HAL_UART_Transmit(&huart2,Interactive_Frame.InteractiveFrameBuffer,(Interactive_Frame.InteractiveFrameBuffer[1] + 9),0xff);
				osDelay(30);
				
				custom_client_data.key_value1=0;
				custom_client_data.key_value2=0;
				custom_client_data.x_position=0;	
				custom_client_data.mouse_left=0;
				custom_client_data.y_position=0;
				custom_client_data.mouse_right=0;
				memcpy(Interactive_Frame.InteractiveFrameBuffer+JUDGE_DATA_OFFSET,&custom_client_data,sizeof(custom_client_data_t));
				Append_CRC_Check_Sum(Interactive_Frame.InteractiveFrameBuffer,Interactive_Frame.InteractiveFrameBuffer[1]);//���ݶεĳ�����8
				HAL_UART_Transmit(&huart2,Interactive_Frame.InteractiveFrameBuffer,(Interactive_Frame.InteractiveFrameBuffer[1] + 9),0xff);
				osDelay(150);
			
				key3_num --;
			}
			
			osDelay(1);
 
		}
		
}

