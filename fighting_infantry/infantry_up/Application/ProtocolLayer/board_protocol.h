#ifndef  __BOARD_PROTOCOL_H
#define  __BOARD_PROTOCOL_H

#include "stdint.h"
#include <stdbool.h>

/**
  * @brief  上板发给下板信息结构体
  *
  */
typedef struct __attribute__((packed))
{
	uint8_t SOF;                 // 帧头，数据帧的起始标志
  uint8_t CRC8;                // 循环冗余校验，用于校验帧头部分的数据完整性
	
	//云台
	float pitch_imu;             //pitch轴陀螺仪角度       
	float yaw_imu;               //yaw轴陀螺仪角度                     
	float yaw_v;                 //yaw轴陀螺仪速度              
	float pitch_v;               //pitch轴陀螺仪速度              
	float pitch_mec;             //pitch轴机械角度            
	
	//发射机构
	float dial_angle_target;    
	float dial_speed_target;    
	float dial_current_target;  
	uint8_t is_dial_need_sleep; //拨盘是否需要睡眠
	uint8_t dial_mode;          //拨盘模式，单发 0，连发 1
	
	//视觉信息
	uint8_t vision_state;       //视觉状态
	uint8_t is_hit_now;         //拨盘能否立即打弹，能为 1，不能为 0
	uint8_t is_find_Target;     //有无找到目标
	uint8_t is_find_dafu;       //有无发现打符
	uint8_t is_find_base;       //有无找到基地
	uint8_t is_find_outpost;    //有无发现前哨
	
	uint16_t launch_timing;     //发射延迟，视觉预判
	
	
	float vision_pitch_tar;     //自瞄pitch目标值
	float vision_yaw_tar;       //自瞄yaw目标值
	
	uint32_t reserved;          //保留位
	
	uint16_t CRC16;             // 循环冗余校验，用于校验整个数据帧的完整性
	
}C_Board_Tx_Pkt_t;


/**
  * @brief  下板发给上板信息结构体
  *
  */

typedef struct __attribute__((packed))
{
	uint8_t SOF;                //帧头，数据帧的起始标志
  uint8_t CRC8;               //循环冗余校验，用于校验帧头部分的数据完整性
	
	//整车
	uint8_t car_state;          //整车状态,掉线为 0，遥控在线为 1，键鼠在线为 2
	
	//云台
	uint8_t Gimbal_state;       //云台上线为 1，下线 0
	uint8_t Gimbal_mode;        //陀螺仪模式 0，小陀螺模式 1，机械模式 2，吊射（机械模式）3
	
	float pitch_imu_tar;        //pitch陀螺仪模式目标角度
	float yaw_imu_tar;
	float pitch_mec_tar;        //pitch机械模式目标角度
	uint8_t yaw_offset;         //ywa轴发射后角度偏移
	
	//发射机构
  uint16_t dial_angle;
	int16_t dial_speed;
	int16_t dial_current;
	uint8_t is_dial_online;     //拨盘是否在线    
  uint8_t is_dial_self_reset; //键鼠时拨盘是否自动复位	
	
	uint8_t Launch_state;       //发射机构状态
	uint8_t Launch_mode;        //发射机构模式,单发为 0，连发为 1
	uint8_t is_fire;            //操作手是否开火，不开火为 0，开火为 1

	//裁判系统
	float bullet_speed;              //当前弹速  
	float firing_freq;               //射频      
	float muzzle_temp;               //枪口温度  
	uint16_t allow_bullet_cnt;       //允许发弹量
	float  muzzle_temp_max;          //枪口热量上限   
	
	//视觉信息
	uint8_t my_color;           //我的颜色
	uint8_t is_video_open;      //图传是否打开
	uint8_t vision_mode;        //视觉模式，开自瞄为 1，否则为 0
	uint8_t is_operater_ctrl;   //自瞄下是否操作手介入
	uint8_t auto_target;        //0 车，1 前哨，2 小符，3 大符
	uint8_t blood_0;            //英雄
	uint8_t blood_1;            //工程
	uint8_t blood_2;            //哨兵
	uint8_t blood_3;            //步兵
	uint8_t blood_4;            //无人机
	uint8_t blood_5;            //雷达
	uint8_t blood_6;            //基地
	uint8_t blood_7;            //前哨
	float v_x;                  //底盘分解后的x轴速度，用于跑打
	float v_y;                  //底盘分解后的y轴速度
	
	
	
	uint32_t reserved; 
	
	uint16_t CRC16;             // 循环冗余校验，用于校验整个数据帧的完整性
	 
}C_Board_Rx_Info_t;

extern C_Board_Tx_Pkt_t  C_Board_Tx_Pkt;
extern C_Board_Rx_Info_t C_Board_Rx_Info;


bool C_Board_Tx_Data(C_Board_Tx_Pkt_t* C_Board_Tx_Pkt);
bool C_Board_Rx_Data(C_Board_Rx_Info_t* C_Board_Rx_Info,uint8_t *rxBuf);


#endif

