#ifndef  __VISION_PROTOCOL_H
#define  __VISION_PROTOCOL_H

#include "stdint.h"
#include <stdbool.h>

/**
 * @brief 电控发给视觉的数据帧结构体
 */
typedef struct  __attribute__((packed)) 
{
	uint8_t SOF;                            // 帧头，数据帧的起始标志
  uint8_t CRC8;                           // 循环冗余校验，用于校验帧头部分的数据完整性
 
  uint8_t own_color;                      // 己方颜色，0红 1蓝
  uint8_t game_start;                     // 比赛开始
  uint8_t is_ready;                       // 是否允许打弹（热量够 && 复位完毕）
  uint8_t mode;                           //1自瞄，2小符，3大符，4前哨，5英雄

  float yaw;                              // 当前yaw角
  float pitch;                            // 当前pitch角
  float roll;                             // 当前云台roll角
  float yaw_speed;                        // yaw轴速度
  float pitch_speed;                      // pitch轴速度 
	float x_v;                              //车的x方向速度
	float y_v;                              //车的y方向速度
	
	uint8_t blood[8];                       //敌方各个兵种的血量
		
  int8_t pitch_offset;                    // pitch轴偏移量（电控退自瞄后清零）
  int8_t yaw_offset;                      // yaw轴偏移量（电控退自瞄后清零）
	
  float bullet_speed;                     // 子弹速度

  uint32_t user_debug;                    // 用户调试信息：
                                          // - 单发模式：接收命令到子弹过测速模块的延时（ms）
                                          // - 连发模式：发射子弹的间隔时间（ms）
                                          // - 通用：用于调试目的
  uint16_t CRC16;                         // 循环冗余校验，用于校验整个数据帧的完整性
} ElectricalToVisionFrame;


/**
 * @brief 视觉发给电控的数据帧结构体
 */
typedef struct  __attribute__((packed)) 
{
  uint8_t SOF;                            // 帧头，数据帧的起始标志
  uint8_t CRC8;                           // 循环冗余校验，用于校验帧头部分的数据完整性
  uint32_t all_flags;                     // 整体32位标志值
  /*
     is_find_target : 1;                  // 位0：用于决定是否给视觉控pitch、yaw
     is_keep_shooting : 1;                // 位1：用于拨盘速度环还是角度环
     is_enable_shootting : 1;             // 位2：用于是否可以打弹
	   detect_num : 4;                      // 位3-6：锁到装甲板几号
	   mode : 3;
     reserved : 22;                       // 位7-31：保留位
 */
  float yaw;                              // 目标yaw角
  float pitch;                            // 目标pitch角
	
	uint8_t ui_x;                           //电控绘制的ui的x坐标
	uint8_t ui_y;                           //电控绘制的ui的y坐标
	
	uint8_t is_find_buff;                   //是否发现能量机关
 
  uint32_t user_debug;                    // 用户调试信息，自定义debug
  uint16_t CRC16;                         // 循环冗余校验，用于校验整个数据帧的完整性
} VisionToElectricalFrame;


extern ElectricalToVisionFrame vision_tx_frame;
extern VisionToElectricalFrame vision_rx_frame;
extern uint8_t TxBuf[54];
extern uint16_t vision_cnt;

bool Vision_Tx_data(ElectricalToVisionFrame* vision_tx_frame);
bool Vision_Rx_Data(VisionToElectricalFrame* vision_rx_frame,uint8_t *rxBuf);
void Vision_heart_beat(void);		

#endif


