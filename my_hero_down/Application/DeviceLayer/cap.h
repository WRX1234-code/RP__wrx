#ifndef __CAP_H
#define __CAP_H

#include "main.h"

#define CAP_OUTPUT_POWER_LIMIT 300
#define MASTER_CAN_RX_ID 0x211
#define CAPBOARD_CAN_TX_ID 0x222
#define WIRELESS_CHARGE_CAN_RX_ID 0x212
/* 通信相关结构体 */
typedef struct __attribute__((packed)) rx_info_struct {
    
    int16_t now_chassis_power;              // 当前底盘消耗功率
    int16_t now_cap_V;                      // 当前电容组电压
    int16_t now_cap_I;                      // 当前电容组电流
    
    struct __attribute__((packed)) bit_state_struct
    {
        uint8_t ability             : 1;    // 电容是否有放电能力，0为无，1为有
        uint8_t is_in_pre_charge_mode : 1;  // 是否使能预充电模式，0为不使能，1为使能
		uint8_t unuse               : 6;    // 暂时未使用
    }bit_state;
    
} capboard_rx_info_t;

typedef struct __attribute__((packed)) tx_info_struct {
    
    uint8_t  chassis_power_buffer;          // 底盘能量缓冲（在buffer_pid_ctrl中被使用）
    uint16_t chassis_power_limit ;          // 机器人底盘功率限制上限（在power_pid_ctrl中被使用）
    int16_t  cap_power_out_limit ;          // 电容放电功率限制，定义为负值（在cap_i_pid_ctrl中被使用）
    uint16_t cap_power_in_limit  ;          // 电容充电功率限制，定义为正值（在cap_i_pid_ctrl中被使用）
    
    struct __attribute__((packed)) bit_control_struct
    {
        uint8_t cap_switch : 1;             // 电容开关，1为开，0为关（在check_states中的初始化状态机中被使用）
        uint8_t turbo_mode : 1;             // 是否使用缓冲能量来充电，0为不用，1为用（在integrate_pid_ctrl中被使用）
        uint8_t pre_charge_mode_en : 1; // 是否使能预充电模式，0为不使能，1为使能
		uint8_t unuse      : 5;             // 暂时未使用
    }bit_control;
    
} capboard_tx_info_t;

typedef struct __attribute__((packed)){
 int16_t charging_power; // 充电功率
 uint8_t is_charging; // 是否在充电，1为充电，0为不充电
 uint8_t reserved1;
 uint16_t reserved2; // 保留字段，暂时未使用
 uint16_t reserved3; 
} wireless_rx_info_t;


typedef enum {
	
	CAP_ONLINE,
	CAP_OFFLINE,

} cap_state_e;

typedef enum {
	WIRELESS_CHARGE_ONLINE,
	WIRELESS_CHARGE_OFFLINE,
} wireless_charge_state_e;

typedef enum {
	PACK_HAVE_NOT_UPDATED, //未更新
	PACK_HAVE_UPDATED,		 //已更新
} cap_pack_state_e;

typedef struct cap_info_struct {
	
	uint16_t	can_rx_Id;//超电->主控
	uint16_t	can_tx_Id;//主控->超电
	uint16_t	wireless_can_rx_Id;//无线充->主控
	capboard_tx_info_t     cap_tx_data;
	capboard_rx_info_t     cap_rx_data;
	wireless_rx_info_t    wireless_rx_data;
	uint8_t   cap_offline_cnt;
	uint8_t   wireless_offline_cnt;
	uint8_t   offline_max_cnt;

} cap_info_t;


typedef struct cap_struct{

	cap_info_t 	info;

	void				(*setdata)(struct cap_struct *self,uint16_t powerBuff,uint16_t powerLimit);
	void				(*update)(struct cap_struct *self,uint32_t canId, uint8_t *rxBuf);
	void				(*heart_beat)(struct cap_struct *self);
	
	float cap_U;
	float cap_I;
	float charging_power; // 充电功率

	cap_state_e	 state;
	wireless_charge_state_e wireless_charge_state;
	cap_pack_state_e judge_pack_state;
}cap_t;

extern cap_t cap;

#endif

