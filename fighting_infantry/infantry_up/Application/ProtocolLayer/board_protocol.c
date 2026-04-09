#include "board_protocol.h"
#include "string.h"
#include <stdbool.h>
#include "crc.h"
#include "usart.h"
#include "rp_math.h"
#include "drv_can.h"

C_Board_Tx_Pkt_t C_Board_Tx_Pkt = {
	.SOF = 0xA6,

};

C_Board_Rx_Info_t C_Board_Rx_Info;

uint8_t C_Board_TxBuf[58];
uint8_t board_cnt = 0;


bool C_Board_Tx_Data(C_Board_Tx_Pkt_t* C_Board_Tx_Pkt)
{
	memcpy(C_Board_TxBuf, C_Board_Tx_Pkt, sizeof(C_Board_Tx_Pkt_t));
	
	Append_CRC8_Check_Sum(C_Board_TxBuf, 3);
	Append_CRC16_Check_Sum(C_Board_TxBuf, sizeof(C_Board_Tx_Pkt_t));
	
	if(HAL_UART_Transmit_DMA(&huart6,C_Board_TxBuf,sizeof(C_Board_Tx_Pkt_t)) == HAL_OK)
	{
		return true;
	}
	
	return false;
}


bool C_Board_Rx_Data(C_Board_Rx_Info_t* C_Board_Rx_Info,uint8_t *rxBuf)
{
	if(rxBuf[0] == 0xA7)
	{
		if(Verify_CRC8_Check_Sum(rxBuf, 3) == true)
		{
			if(Verify_CRC16_Check_Sum(rxBuf, sizeof(C_Board_Rx_Info_t)) == true)
			{
				memcpy(C_Board_Rx_Info, rxBuf, sizeof(C_Board_Rx_Info_t));
			
				return true;
			}
		}
	}
	return false;
}

uint8_t tx_pkt1[8];
uint8_t tx_pkt2[8];
uint8_t tx_pkt3[8];

void C_Board_Tx1(void)
{
	
	uint16_t t1,t2,t3,t4;

  t1 = float_to_uint16(C_Board_Tx_Pkt.pitch_imu,-360.f,360.f,16);   //pitch轴陀螺仪角度    
	t2 = float_to_uint16(C_Board_Tx_Pkt.yaw_imu,-360.f,360.f,16);     //yaw轴陀螺仪角度      
	t3 = float_to_uint16(C_Board_Tx_Pkt.yaw_v,-5000.f,5000.f,16);     //yaw轴陀螺仪速度      
	t4 = float_to_uint16(C_Board_Tx_Pkt.pitch_v,-5000.f,5000.f,16);   //pitch轴陀螺仪速度    
	
	tx_pkt1[0] = t1 >> 8;
	tx_pkt1[1] = t1;
	tx_pkt1[2] = t2 >> 8;
	tx_pkt1[3] = t2;
	tx_pkt1[4] = t3 >> 8;
	tx_pkt1[5] = t3;
	tx_pkt1[6] = t4 >> 8;
	tx_pkt1[7] = t4;
	
	CAN2_SendData(0xC1, tx_pkt1);
}

void C_Board_Tx2(void)
{
	
	tx_pkt2[0] = C_Board_Tx_Pkt.dial_angle_target >> 24;
	tx_pkt2[1] = C_Board_Tx_Pkt.dial_angle_target >> 16;
	tx_pkt2[2] = C_Board_Tx_Pkt.dial_angle_target >> 8;
	tx_pkt2[3] = C_Board_Tx_Pkt.dial_angle_target;
	tx_pkt2[4] = C_Board_Tx_Pkt.dial_speed_target >> 8;
	tx_pkt2[5] = C_Board_Tx_Pkt.dial_speed_target;
	tx_pkt2[6] = C_Board_Tx_Pkt.dial_current_target >> 8;
	tx_pkt2[7] = C_Board_Tx_Pkt.dial_current_target;
	
	CAN2_SendData(0xC2, tx_pkt2);
}

void C_Board_Tx3(void)
{
	memset(tx_pkt3,0,8);
	uint16_t t1,t2,t3;
	
	t1 = float_to_uint16(C_Board_Tx_Pkt.vision_pitch_tar,-180.f,180.f,16);   //pitch轴陀螺仪角度    
	t2 = float_to_uint16(C_Board_Tx_Pkt.vision_yaw_tar,-180.f,180.f,16);     //yaw轴陀螺仪角度      
	t3 = float_to_uint16(C_Board_Tx_Pkt.pitch_mec,-3.14f,3.14f,16);    
	
	tx_pkt3[0] |= (C_Board_Tx_Pkt.vision_state & 0x01) << 0;//视觉状态
	tx_pkt3[0] |= (C_Board_Tx_Pkt.is_hit_now & 0x01) << 1;//拨盘能否立即打弹，能为 1，不能为 0
	tx_pkt3[0] |= (C_Board_Tx_Pkt.is_find_Target & 0x01) << 2;//有无找到目标
	tx_pkt3[0] |= (C_Board_Tx_Pkt.is_find_dafu & 0x01) << 3;//有无发现打符
	tx_pkt3[0] |= (C_Board_Tx_Pkt.is_find_base & 0x01) << 4;//有无找到基地
	tx_pkt3[0] |= (C_Board_Tx_Pkt.is_find_outpost & 0x01) << 5;//有无发现前哨
	tx_pkt3[0] |= (C_Board_Tx_Pkt.is_dial_need_sleep & 0x01) << 6;//拨盘是否需要睡眠
	tx_pkt3[0] |= (C_Board_Tx_Pkt.dial_mode & 0x01) << 7;//拨盘模式，单发 0，连发 1
				
	tx_pkt3[1] = C_Board_Tx_Pkt. launch_timing; //发射延迟，视觉预判
	
	tx_pkt3[2] = t1 >> 8;//自瞄pitch目标值
	tx_pkt3[3] = t1;
	tx_pkt3[4] = t2 >> 8; //自瞄yaw目标值
	tx_pkt3[5] = t2;
	tx_pkt3[6] = t3 >> 8; //pitch轴机械角度     
	tx_pkt3[7] = t3;
	
	
//	tx_pkt3[2] = C_Board_Tx_Pkt. launch_timing;
//	tx_pkt3[3] = t1 >> 8;//自瞄pitch目标值
//	tx_pkt3[4] = t1;
//	tx_pkt3[5] = t2 >> 8; //自瞄yaw目标值
//	tx_pkt3[6] = t2;
//	tx_pkt3[7] = 0;
	
	CAN2_SendData(0xC3, tx_pkt3);
}


void C_Board_Rx1(uint8_t* rxbuf)
{
	uint16_t t1,t2;

    C_Board_Rx_Info.car_state      = (rxbuf[0] >> 0) & 0x03;  
    C_Board_Rx_Info.Gimbal_state   = (rxbuf[0] >> 2) & 0x01;  
    C_Board_Rx_Info.Gimbal_mode    = (rxbuf[0] >> 3) & 0x01;  
    C_Board_Rx_Info.Launch_state   = (rxbuf[0] >> 4) & 0x01;  
    C_Board_Rx_Info.Launch_mode    = (rxbuf[0] >> 5) & 0x01;  
    C_Board_Rx_Info.my_color       = (rxbuf[0] >> 6) & 0x01;  
    C_Board_Rx_Info.is_video_open  = (rxbuf[0] >> 7) & 0x01;  
    
    C_Board_Rx_Info.vision_mode    = (rxbuf[1] >> 0) & 0x03;
 
    C_Board_Rx_Info.is_fire        = (rxbuf[1] >> 3) & 0x01;  
    C_Board_Rx_Info.is_dial_online    = (rxbuf[1] >> 4) & 0x01;
    C_Board_Rx_Info.is_dial_self_reset  = (rxbuf[1] >> 5 ) & 0x01;
    
   
    C_Board_Rx_Info.allow_bullet_cnt = ((uint16_t)rxbuf[2] << 8) | rxbuf[3];
	
    t1 = ((uint16_t)rxbuf[4] << 8) | rxbuf[5];
	  t2 = ((uint16_t)rxbuf[6] << 8) | rxbuf[7];
		 
		C_Board_Rx_Info.v_x = uint16_to_float(t1, -5000.f, 5000.f, 16);
		C_Board_Rx_Info.v_y = uint16_to_float(t2, -5000.f, 5000.f, 16);
    
    board_cnt = 0;
}

void C_Board_Rx2(uint8_t* rxbuf)
{
	uint16_t t1 = ((uint16_t)rxbuf[0] << 8) | rxbuf[1];
    uint16_t t2 = ((uint16_t)rxbuf[2] << 8) | rxbuf[3];
    uint16_t t3 = ((uint16_t)rxbuf[4] << 8) | rxbuf[5];
    uint16_t t4 = ((uint16_t)rxbuf[6] << 8) | rxbuf[7];
    
    C_Board_Rx_Info.pitch_imu_tar = uint16_to_float(t1, -360.0f, 360.0f, 16);
    C_Board_Rx_Info.yaw_imu_tar   = uint16_to_float(t2, -360.0f, 360.0f, 16);
    C_Board_Rx_Info.pitch_mec_tar = uint16_to_float(t3, -3.14f, 3.14f, 16);
    C_Board_Rx_Info.yaw_offset    = uint16_to_float(t4, -3.14f, 3.14f, 16);
	
	  board_cnt = 0;
}
void C_Board_Rx3(uint8_t* rxbuf)
{
	uint16_t t1 = ((uint16_t)rxbuf[0] << 8) | rxbuf[1];
    C_Board_Rx_Info.bullet_speed = uint16_to_float(t1, -50.0f, 50.0f, 16);
    

    uint16_t t2 = ((uint16_t)rxbuf[2] << 8) | rxbuf[3];
    C_Board_Rx_Info.firing_freq = uint16_to_float(t2, -50.0f, 50.0f, 16);
    
  
    C_Board_Rx_Info.muzzle_temp = ((uint16_t)rxbuf[4] << 8) | rxbuf[5];
    
 
    C_Board_Rx_Info.muzzle_temp_max = ((uint16_t)rxbuf[6] << 8) | rxbuf[7];
	
	board_cnt = 0;
}
void C_Board_Rx4(uint8_t* rxbuf)
{
	C_Board_Rx_Info.blood_0 = rxbuf[0];
    C_Board_Rx_Info.blood_1 = rxbuf[1];
    C_Board_Rx_Info.blood_2 = rxbuf[2];
    C_Board_Rx_Info.blood_3 = rxbuf[3];
    C_Board_Rx_Info.blood_4 = rxbuf[4];
    C_Board_Rx_Info.blood_5 = rxbuf[5];
    C_Board_Rx_Info.blood_6 = rxbuf[6];
    C_Board_Rx_Info.blood_7 = rxbuf[7];
	
	board_cnt = 0;
}
void C_Board_Rx5(uint8_t* rxbuf)
{
	C_Board_Rx_Info.dial_angle = ((uint32_t)rxbuf[0] << 24) |
                                 ((uint32_t)rxbuf[1] << 16) |
                                 ((uint32_t)rxbuf[2] << 8)  |
                                 rxbuf[3];
    

    int16_t speed = (int16_t)(((uint16_t)rxbuf[4] << 8) | rxbuf[5]);
    C_Board_Rx_Info.dial_speed = speed;
    

    int16_t current = (int16_t)(((uint16_t)rxbuf[6] << 8) | rxbuf[7]);
    C_Board_Rx_Info.dial_current = current;
	
	board_cnt = 0;
}


void Board_Heart_Beat(void)
{
	board_cnt ++;
	
	if(board_cnt >= 70)
	{
		board_cnt = 70;
	}
}





