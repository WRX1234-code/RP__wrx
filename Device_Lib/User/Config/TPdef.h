#ifndef __TPdef_H__
#define __TPdef_H__

#include "main.h"

#define JUDGE_DATA_LENGTH(n) (n + 9)
#define JUDGE_CRC8_OFFSET (4)
#define JUDGE_SOF_OFFSET (0)
#define JUDGE_DATALENGTH_OFFSET (1)
#define JUDGE_SEQ_OFFSET (3)
#define JUDGE_CMDID_OFFSET (5)
#define JUDGE_DATA_OFFSET (7)
#define JUDGE_CRC16_OFFSET(n) (n + JUDGE_DATA_OFFSET)
/*
 *	|帧格式
 *	|frame_header (5-byte)
 *	|cmd_id (2-byte)
 *	|data (n-byte)
 *	|frame_tail (2-byte,CRC16,整包校验)
 *
 *	|Frame header 格式:
 *	|SOF    data_length seq     CRC8
 *	|1-byte 2-byte      1-byte  1-byte
 */


typedef  __packed struct
{
   // uint16_t key_value;
   	uint8_t  key_value1;
    uint8_t  key_value2;
    uint16_t x_position:12;
    uint16_t mouse_left:4;
    uint16_t y_position:12;
    uint16_t mouse_right:4;
    uint16_t reserved;
} custom_client_data_t;


typedef __packed struct
{
	uint8_t SOF;
	__packed union
	{
		uint16_t Frame;
		uint8_t FrameBuffer[2];
	} data_length;
	uint8_t seq;
	uint8_t CRC8;
} frame_header_t;
/*
这里是把帧头,cmd_id与数据合起来
数据长度为121
但是还没有帧尾
*/
typedef __packed struct
{
	__packed union
	{
		frame_header_t Frame;
		uint8_t FrameBuffer[5];
	} frame_header;

	__packed union
	{
		uint16_t Frame;
		uint8_t FrameBuffer[2];
	} cmd_id;

	uint8_t data[8];
} Interactive_Frame_t;

void Interactive_Frame_Init(void);
 
 
 
 /*-------------------------------------------CRC校验---------------------------------------------------*/
/**
  * @brief  裁判系统数据校验
  * @param  __RECEIVEBUFFER__：  接收到的裁判系统数据头帧所在地址
  * @param  __DATALENGTH__：     一帧数据内的数据量/Bytes（内容）
  * @retval 1：                  校验正确
  * @retval 0：                  校验错误
  * @note	None
  */
#define Verify_CRC_Check_Sum(__RECEIVEBUFFER__, __DATALENGTH__) (Verify_CRC8_Check_Sum(__RECEIVEBUFFER__, JUDGE_CRC8_OFFSET + 1) && Verify_CRC16_Check_Sum(__RECEIVEBUFFER__, __DATALENGTH__ + JUDGE_DATA_LENGTH(0)))

/**
  * @brief  裁判系统添加校验
  * @param  __TRANSMITBUFFER__： 发送到裁判系统的数据中头帧所在地址
  * @param  __DATALENGTH__：     一帧数据内的数据量/Bytes（内容）
  * @retval None
  * @note	None
  */
#define Append_CRC_Check_Sum(__TRANSMITBUFFER__, __DATALENGTH__)                       \
do                                                                                     \
{                                                                                      \
    Append_CRC8_Check_Sum(__TRANSMITBUFFER__, JUDGE_CRC8_OFFSET + 1);                  \
    Append_CRC16_Check_Sum(__TRANSMITBUFFER__, __DATALENGTH__ + JUDGE_DATA_LENGTH(0)); \
} while (0U)

/*--------------------------------------------------校验函数--------------------------------------------------*/
unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

/*--------------------------------------------------校验函数--------------------------------------------------*/
#endif
