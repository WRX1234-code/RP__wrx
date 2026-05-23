#ifndef __BOARD_PROTOCOL_H
#define __BOARD_PROTOCOL_H

#include "stdint.h"

#define  BOARD_OFFLINE_CNT_MAX    500

#define  ID_PKT_01     0xC1
#define  ID_PKT_02     0xC2
#define  ID_PKT_03     0xC3
#define  ID_PKT_04     0xC4
#define  ID_PKT_05     0xC5
#define  ID_MEG_01     0xD1
#define  ID_MEG_02     0xD2
#define  ID_MEG_03     0xD3
#define  ID_MEG_04     0xD4
#define  ID_MEG_05     0xD5

typedef struct{
  uint8_t  car_state;       //0是卸力，1是遥控，2是键鼠
//	uint8_t  gimbal_state;    //0是卸力，1是有力
//	uint8_t  launch_state;    //0是关发射机构，1是开发射机构
  uint8_t  car_mode;        //0是机械，1是陀螺,2是狗洞
	uint8_t  vision_mode;     //0无视觉模式，1是普通自瞄，2是小符，3是大符，4是前哨，5是英雄
	uint8_t  game_start;
	uint8_t  my_color;
	
}Board_State_Pkt_t;


typedef struct{
  float  shoot_speed;
	float  shoot_freq;
	uint16_t  shoot_heat_max;
	uint16_t  shoot_heat;
	
}Board_Judge_Shoot_Pkt_t;


typedef struct{
	float yaw_mec_tar;
	float yaw_imu_tar;
	float pitch_mec_tar;
	float pitch_imu_tar;
	
}Board_Gimbal_Target_Pkt_t;

typedef struct{
	uint8_t  launch_state;    //0是关发射机构，1是开发射机构
  uint8_t  shoot_mode;
	uint8_t  shoot_level;


}Board_Shoot_Pkt_t;


typedef struct{
	uint8_t blood_0;            //英雄
	uint8_t blood_1;            //工程
	uint8_t blood_2;            //哨兵
	uint8_t blood_3;            //步兵
	uint8_t blood_4;            //无人机
	uint8_t blood_5;            //雷达
	uint8_t blood_6;            //基地
	uint8_t blood_7;            //前哨

}Board_Blood_Pkt_t;


typedef struct{
  Board_State_Pkt_t                state_pkt;
  Board_Judge_Shoot_Pkt_t          judge_shoot_pkt; 
  Board_Gimbal_Target_Pkt_t        gimbal_target_pkt;
  Board_Shoot_Pkt_t                shoot_pkt;
	Board_Blood_Pkt_t                blood_pkt;

}Board_Tx_Pkt_t;



typedef struct{
	float yaw_mec;
	float yaw_imu;
	float pitch_mec;
	float pitch_imu;

}Board_Gimbal_Meg_t;


typedef struct{
  uint8_t  gimbal_state;
	uint8_t  launch_state;
	uint8_t  vision_state;

}Board_State_Meg_t;


typedef struct{
	Board_Gimbal_Meg_t    gimbal_meg;
	Board_State_Meg_t     state_meg;
	
}Board_Rx_Meg_t;


typedef  struct{
	uint16_t offline_cnt_max;
	uint8_t status;
	uint16_t offline_cnt;

}Board_Status_t;

typedef struct Board_Struct_t{
	Board_Status_t*    status;
	Board_Tx_Pkt_t*    tx_pkt;
  Board_Rx_Meg_t*    rx_meg;
	
	void (*heartbeat)(struct Board_Struct_t *board);
	
	void (*tx_01)(struct Board_Struct_t* board);
	void (*tx_02)(struct Board_Struct_t* board);
	void (*tx_03)(struct Board_Struct_t* board);
	void (*tx_04)(struct Board_Struct_t* board);
	
	void (*rx_01)(struct Board_Struct_t* board, uint8_t *rxBuf);
	void (*rx_02)(struct Board_Struct_t* board, uint8_t *rxBuf);

	void (*init)(struct Board_Struct_t *board);

}Board_t;



extern  Board_t  board;

void Board_Init(Board_t* board);

void Board_Heart_Beat(Board_t* board);

void Board_Tx_Pkt_01(Board_t* board);

void Board_Tx_Pkt_02(Board_t* board);

void Board_Tx_Pkt_03(Board_t* board);

void Board_Tx_Pkt_04(Board_t* board);

void Board_Rx_Meg_01(Board_t* board,uint8_t* rxbuf);

void Board_Rx_Meg_02(Board_t* board,uint8_t* rxbuf);


#endif


