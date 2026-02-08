#include "board_protocol.h"
#include "string.h"
#include <stdbool.h>
#include "crc.h"
#include "usart.h"
#include "rp_math.h"
#include "drv_can.h"

uint16_t Board_cnt;
extern UART_HandleTypeDef huart10;

D_Board_Tx_Pkt_t D_Board_Tx_Pkt = {
	.SOF = 0xA7,

};

D_Board_Rx_Info_t D_Board_Rx_Info;

uint8_t D_Board_TxBuf[74];


bool D_Board_Tx_Data(D_Board_Tx_Pkt_t* D_Board_Tx_Pkt)
{
	memcpy(D_Board_TxBuf, D_Board_Tx_Pkt, sizeof(D_Board_Tx_Pkt_t));
	
	Append_CRC8_Check_Sum(D_Board_TxBuf, 3);
	Append_CRC16_Check_Sum(D_Board_TxBuf, sizeof(D_Board_Tx_Pkt_t));
	
	if(HAL_UART_Transmit_DMA(&huart10,D_Board_TxBuf,sizeof(D_Board_Tx_Pkt_t)) == HAL_OK)
	{
		return true;
	}
	
	return false;
}


bool D_Board_Rx_Data(D_Board_Rx_Info_t* D_Board_Rx_Info,uint8_t *rxBuf)
{
	if(rxBuf[0] == 0xA6)
	{
		if(Verify_CRC8_Check_Sum(rxBuf, 3) == true)
		{
			if(Verify_CRC16_Check_Sum(rxBuf, sizeof(D_Board_Rx_Info_t)) == true)
			{
				memcpy(D_Board_Rx_Info, rxBuf, sizeof(D_Board_Rx_Info_t));
			  Board_cnt = 0;
				return true;
			}
		}
	}
	return false;
}

void D_Board_Heart_Beat(void)
{
//	if(D_Board_Tx_Data(&D_Board_Tx_Pkt) == true)
//	{
//		Board_cnt ++;
//	}
//	if(Board_cnt >= OFFLINE_CNT_MAX)
//	{
//		Board_cnt = OFFLINE_CNT_MAX;
//	}
	if(Board_cnt >= OFFLINE_CNT_MAX)
	{
 	  Board_cnt = OFFLINE_CNT_MAX;
  }
}
	
	
	
uint8_t tx_pkt1[8];
uint8_t tx_pkt2[8];
uint8_t tx_pkt3[8];
uint8_t tx_pkt4[8];
uint8_t tx_pkt5[8];




void D_Board_Tx1(void)
{
	memset(tx_pkt1, 0, 8);
	
	uint8_t t1,t2;

	tx_pkt1[0] |= (D_Board_Tx_Pkt.car_state & 0x03) << 0;
	tx_pkt1[0] |= (D_Board_Tx_Pkt.Gimbal_state & 0x01) << 2;//云台上线 1，下线 0
	tx_pkt1[0] |= (D_Board_Tx_Pkt.Gimbal_mode & 0x01) << 3;//陀螺仪模式 0，小陀螺模式 1，机械模式 2，吊射（机械模式） 3
	tx_pkt1[0] |= (D_Board_Tx_Pkt.Launch_state & 0x01) << 4;//发射机构状态
	tx_pkt1[0] |= (D_Board_Tx_Pkt.Launch_mode & 0x01) << 5;//发射机构模式，单发为 0，连发为 1
	tx_pkt1[0] |= (D_Board_Tx_Pkt.my_color & 0x01) << 6;//我的颜色
	tx_pkt1[0] |= (D_Board_Tx_Pkt.is_video_open & 0x01) << 7;//图传是否打开
	
	
	tx_pkt1[1] |= (D_Board_Tx_Pkt.vision_mode & 0x03) << 0;//视觉模式,0 不开自瞄，1 前哨，2 小符，3 大符，4 英雄

	tx_pkt1[1] |= (D_Board_Tx_Pkt.is_fire & 0x01) << 3;
	tx_pkt1[1] |= (D_Board_Tx_Pkt.is_dial_online & 0x01) << 4;//拨盘是否在线   
	tx_pkt1[1] |= (D_Board_Tx_Pkt.dial_reset & 0x01) << 5;//键鼠时拨盘自动复位，手动命令复位为 1，不复位为 0
	
	
  tx_pkt1[2] = D_Board_Tx_Pkt.allow_bullet_cnt >> 8;
	tx_pkt1[3] = D_Board_Tx_Pkt.allow_bullet_cnt;
	
	t1 = float_to_uint16(D_Board_Tx_Pkt.v_x,-5000.f,5000.f,16);   
	t2 = float_to_uint16(D_Board_Tx_Pkt.v_y,-5000.f,5000.f,16);   
	
	tx_pkt1[4] = t1>>8;
	tx_pkt1[5] = t1;
	tx_pkt1[6] = t2>>8;
	tx_pkt1[7] = t2  ;
	
	CAN_SendData(&hfdcan3, 0xD1, tx_pkt1);
	
	Board_cnt ++;
	
}

void D_Board_Tx2(void)
{
	
	uint16_t t1,t2,t3,t4;
	
	t1 = float_to_uint16(D_Board_Tx_Pkt.pitch_imu_tar,-360.f,360.f,16);      //pitch陀螺仪模式目标角度
	t2 = float_to_uint16(D_Board_Tx_Pkt.yaw_imu_tar,-360.f,360.f,16);  
	t3 = float_to_uint16(D_Board_Tx_Pkt.pitch_mec_tar,-3.14f,3.14f,16);     //pitch机械模式目标角度
	t4 = float_to_uint16(D_Board_Tx_Pkt.yaw_offset,-3.14f,3.14f,16);        //ywa轴发射后角度偏移
	  
	tx_pkt2[0] = t1>>8;
	tx_pkt2[1] = t1;
	tx_pkt2[2] = t2>>8;
	tx_pkt2[3] = t2;
	tx_pkt2[4] = t3>>8;
	tx_pkt2[5] = t3;
	tx_pkt2[6] = t4>>8;
	tx_pkt2[7] = t4;
	
	CAN_SendData(&hfdcan3, 0xD2, tx_pkt2);
	
	Board_cnt ++;
}

void D_Board_Tx3(void)
{       
	
	uint16_t t1,t2;
	
	t1 = float_to_uint16(D_Board_Tx_Pkt.bullet_speed,-50.f,50.f,16);       //当前弹速  
	t2 = float_to_uint16(D_Board_Tx_Pkt.firing_freq,-50.f,50.f,16);        //射频      
	
	tx_pkt3[0] = t1>>8;
	tx_pkt3[1] = t1;
	tx_pkt3[2] = t2>>8;
	tx_pkt3[3] = t2;
	tx_pkt3[4] = D_Board_Tx_Pkt.muzzle_temp>>8;//枪口温度  
	tx_pkt3[5] = D_Board_Tx_Pkt.muzzle_temp;
	tx_pkt3[6] = D_Board_Tx_Pkt.muzzle_temp_max>>8;//枪口热量上限
	tx_pkt3[7] = D_Board_Tx_Pkt.muzzle_temp_max;
	
	CAN_SendData(&hfdcan3, 0xD3, tx_pkt3);
	
	Board_cnt ++;
}

void D_Board_Tx4(void)
{
	tx_pkt4[0] = D_Board_Tx_Pkt.blood_0;//英雄
	tx_pkt4[1] = D_Board_Tx_Pkt.blood_1;//工程
	tx_pkt4[2] = D_Board_Tx_Pkt.blood_2;//哨兵
	tx_pkt4[3] = D_Board_Tx_Pkt.blood_3;//步兵
	tx_pkt4[4] = D_Board_Tx_Pkt.blood_4;//无人机
	tx_pkt4[5] = D_Board_Tx_Pkt.blood_5;//雷达
	tx_pkt4[6] = D_Board_Tx_Pkt.blood_6;//基地
	tx_pkt4[7] = D_Board_Tx_Pkt.blood_7;//前哨
	   
	CAN_SendData(&hfdcan3, 0xD4, tx_pkt4);
	
	Board_cnt ++;
}

void D_Board_Tx5(void)
{
	tx_pkt5[0] = D_Board_Tx_Pkt.dial_angle >> 24;
	tx_pkt5[1] = D_Board_Tx_Pkt.dial_angle >> 16;
	tx_pkt5[2] = D_Board_Tx_Pkt.dial_angle >> 8;
	tx_pkt5[3] = D_Board_Tx_Pkt.dial_angle;
	tx_pkt5[4] = D_Board_Tx_Pkt.dial_speed >> 8;
	tx_pkt5[5] = D_Board_Tx_Pkt.dial_speed;
	tx_pkt5[6] = D_Board_Tx_Pkt.dial_current >> 8;
	tx_pkt5[7] = D_Board_Tx_Pkt.dial_current;
	   
	CAN_SendData(&hfdcan3, 0xD5, tx_pkt5);
	
	Board_cnt ++;
}
void D_Board_Rx1(uint8_t* rxbuf)
{
	uint16_t t1 = ((uint16_t)rxbuf[0] << 8) | rxbuf[1];  
  uint16_t t2 = ((uint16_t)rxbuf[2] << 8) | rxbuf[3];
  uint16_t t3 = ((uint16_t)rxbuf[4] << 8) | rxbuf[5];
  uint16_t t4 = ((uint16_t)rxbuf[6] << 8) | rxbuf[7];
    

  D_Board_Rx_Info.pitch_imu = uint16_to_float(t1, -360.0f, 360.0f,16);
  D_Board_Rx_Info.yaw_imu   = uint16_to_float(t2, -360.0f, 360.0f,16);
  D_Board_Rx_Info.yaw_v     = uint16_to_float(t3, -5000.0f, 5000.0f,16);
  D_Board_Rx_Info.pitch_v   = uint16_to_float(t4, -5000.0f, 5000.0f,16);
	
	  
	Board_cnt = 0;
	
}

void D_Board_Rx2(uint8_t* rxbuf)
{
	D_Board_Rx_Info.dial_angle_target = ((int32_t)rxbuf[0] << 24) |
                                        ((int32_t)rxbuf[1] << 16) |
                                        ((int32_t)rxbuf[2] << 8)  |
                                        rxbuf[3];
    

  D_Board_Rx_Info.dial_speed_target = ((int16_t)rxbuf[4] << 8) | rxbuf[5];
    
 
  D_Board_Rx_Info.dial_current_target = ((int16_t)rxbuf[6] << 8) | rxbuf[7];
	
	Board_cnt = 0;
}


void D_Board_Rx3(uint8_t* rxbuf)
{
		D_Board_Rx_Info.vision_state    = (rxbuf[0] >> 0) & 0x01;
    D_Board_Rx_Info.is_hit_now      = (rxbuf[0] >> 1) & 0x01;
    D_Board_Rx_Info.is_find_Target  = (rxbuf[0] >> 2) & 0x01;
    D_Board_Rx_Info.is_find_dafu    = (rxbuf[0] >> 3) & 0x01;
    D_Board_Rx_Info.is_find_base    = (rxbuf[0] >> 4) & 0x01;
    D_Board_Rx_Info.is_find_outpost = (rxbuf[0] >> 5) & 0x01;
    D_Board_Rx_Info.is_dial_need_sleep = (rxbuf[0] >> 6) & 0x01;
    D_Board_Rx_Info.dial_mode       = (rxbuf[0] >> 7) & 0x01;
    

    D_Board_Rx_Info.launch_timing = rxbuf[1];
    
  
    uint16_t t1 = ((uint16_t)rxbuf[2] << 8) | rxbuf[3];
    D_Board_Rx_Info.vision_pitch_tar = uint16_to_float(t1, -3.14f, 3.14f,16);
    

    uint16_t t2 = ((uint16_t)rxbuf[4] << 8) | rxbuf[5];
    D_Board_Rx_Info.vision_yaw_tar = uint16_to_float(t2, -3.14f, 3.14f,16);
	
	  uint16_t t3 = ((uint16_t)rxbuf[6] << 8) | rxbuf[7];
    D_Board_Rx_Info.pitch_mec = uint16_to_float(t3, -3.14f, 3.14f,16);
	
	  Board_cnt = 0;
	
}











