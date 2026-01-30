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
  __packed union {                        // 状态标志位联合体（32位）
    uint32_t all_flags;                   // 整体32位标志值
    __packed struct {
      uint32_t own_color : 1;             // 位0：己方颜色
      uint32_t game_start : 1;            // 位1：比赛开始
      uint32_t is_ready : 1;              // 位2：是否允许打弹（热量够 && 复位完毕）
      uint32_t outpost_mode : 1;          // 位3：只锁前哨模式
      uint32_t engineer_mode : 1;         // 位4：只锁工程模式
			uint32_t buff_mode : 1;             // 位5：只锁符模式
			uint32_t hero_mode : 1;             // 位6：只锁英雄模式
			uint32_t is_operator_ctrl : 1;      // 位7：自瞄是否操作手介入
//			uint32_t lob_mode : 1;              // 只吊射模式
      uint32_t reserved : 24;             // 位8-31：可扩展
    } bit;                                // 按位访问的子结构
  } flag_union; 
  float yaw;                              // 当前yaw角
  float pitch;                            // 当前pitch角
  float roll;                             // 当前云台roll角
  float yaw_speed;                        // yaw轴速度
  float pitch_speed;                      // pitch轴速度
  int8_t pitch_offset;                    // pitch轴偏移量（电控退自瞄后清零）
  int8_t yaw_offset;                      // yaw轴偏移量（电控退自瞄后清零）
  float bullet_speed;                     // 子弹速度

  uint16_t bullet_id;                     // 每打出一发加1

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
  __packed union {                        // 状态标志位联合体（32位）
    uint32_t all_flags;                   // 整体32位标志值
    __packed struct {
      uint32_t is_find_target : 1;        // 位0：用于决定是否给视觉控pitch、yaw
      uint32_t is_keep_shooting : 1;      // 位1：用于拨盘速度环还是角度环，暂定除了
      uint32_t is_enable_shootting : 1;   // 位2：用于是否可以打弹
      uint32_t detect_num : 4;            // 位3-6：锁到几号（占用4位，支持0-15编号
      uint32_t reserved : 25;             // 位7-31：保留位
    } bit;                                // 按位访问的子结构
  } flag_union; 
  float yaw;                              // 目标yaw角
  float pitch;                            // 目标pitch角
 
 	uint16_t timing;                        //发射延时

 
  uint32_t user_debug;                    // 用户调试信息，自定义debug
  uint16_t CRC16;                         // 循环冗余校验，用于校验整个数据帧的完整性
} VisionToElectricalFrame;


extern ElectricalToVisionFrame vision_tx_frame;
extern VisionToElectricalFrame vision_rx_frame;
extern uint8_t TxBuf[37];

bool Vision_Tx_data(ElectricalToVisionFrame* vision_tx_frame);
bool Vision_Rx_Data(VisionToElectricalFrame* vision_rx_frame,uint8_t *rxBuf);

#endif


