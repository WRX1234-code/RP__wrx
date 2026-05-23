//#ifndef _WL_RX_BOARD_H
//#define _WL_RX_BOARD_H

///* Including Files -------------------------------------------------------------------------------------------------------------------------------------------------- */
//#include "stm32h7xx_hal.h"
//#include "WL_RX_board_config.h"
//#include "config.h"
//#include "pid.h"
////#include "hrtim.h"

///* Macro ------------------------------------------------------------------------------------------------------------------------------------------------------------ */


///* Define Exported Variables Type ----------------------------------------------------------------------------------------------------------------------------------- */

//typedef enum {
//    IN_NORMAL = 0,      // 正常工作
//    IN_WARNING,         // 告警
//    IN_ERROR,           // 工作异常
//} work_state_e;

//typedef enum {
//    DONE = 0,           // 初始化完成
//    NOT,                // 未初始化
//} init_state_e;

//typedef enum {
//    NOTABLE = 0,        // 不可使用
//    ABLE,               // 可使用
//} ability_state_e;

//typedef enum {
//    DISCHARGING = 0,    // 放电中
//    IDLE,               // 空闲
//    CHARGING,           // 充电中
//} using_state_e;

//typedef enum {
//    NORMAL = 0,         // 正常工作
//    UNDER,              // 电压或电流低于正常值
//    OVER,               // 电压或电流高于正常值
//    CUT_OFF,            // 切断
//} V_or_C_state_e;

//typedef enum {
//    MATCHED = 0,        // 当前代码与板号匹配
//    MISMATCHED,         // 当前代码与板号不匹配
//} code_match_state_e;

//typedef enum {
//    CONNECTED = 0,      // 有通讯
//    MISSED,             // 无通讯
//} communicate_master_state_e;

//typedef enum {
//    BUCK = 0,           // 工作在降压模式
//    BUCK_BOOST,         // 工作在等压模式附近
//    BOOST,              // 工作在升压模式
//} pwm_state_e;

//typedef struct WL_RX_board_state_struct {
//    
//    work_state_e     total_state;                   // 电容板总状态
//    init_state_e     init_state;                    // 电容板初始化状态
//    ability_state_e  ability_state;                 // 是否能放电供能
//    using_state_e    using_state;                   // 电容组使用状态
//    
//    V_or_C_state_e   power_supply_state;            // 裁判系统端电源输入状态
//    V_or_C_state_e   cap_voltage_state;             // 电容电压状态
//    V_or_C_state_e   cap_current_state;             // 电容电流状态
//    
//    code_match_state_e  code_match_state;
//    communicate_master_state_e  communicate_state;  // 与主控的通信状态
//    
//    pwm_state_e pwm_state;                          // BUCK-BOOST电路工作模式
//    
//} WL_RX_board_state_t;

//typedef struct capboard_struct {
//    
//    /* 电压电流信息测量量 */
//    uint16_t dma_adc_raw_data[MEASURE_NUM];         // dma传输的MEASURE_NUM个测量参数*EACH_RAW_DATA_NUM个原始数据
//    float    adc_filter_info[MEASURE_NUM];          // MEASURE_NUM个测量参数各自滤波并转换后的值
//    float    adc_cali_info[MEASURE_NUM];            // MEASURE_NUM个测量参数各自校准后的值，是最终使用值
//    
//    /* 电压电流信息计算量 */
//    float input_power;                              // 输入当前功率
//    float cap_power;                                // 电容组当前功率
//    float now_volt_ratio;                           // 当前电容组电压与电源电压之比
//    float percentage_energy;                        // 电容组百分比电量（当前电压/最大电压）

//    float bat_v;
//    float bat_i;
//    float cap_v;
//    float cap_i;
//    
//    /* 电容板状态量 */
//    uint32_t device_unique_id[3];                   // 芯片特征id
//    WL_RX_board_state_t WL_RX_board_state;                // 电容组状态汇总
//    
//    /* 各参数pid控制结构体 */
//    incremental_pid_ctrl_t pid_power;               // 总输入功率pid控制结构体
//    incremental_pid_ctrl_t pid_cap_i;               // 电容组电流pid控制结构体
//    
//    /* 实际输出量 */
//    volatile float final_out_volt_ratio;            // 最终输出电压比率（仅在pid最终输出或初始化时被赋值）
//    volatile uint16_t final_TIMERB_cmp1;            // 实际写入TIMERB的比较值1（仅在pwm占空比计算或进保护时被赋值）
//    volatile uint16_t final_TIMERB_cmp3;            // 实际写入TIMERB的比较值3（仅在pwm占空比计算或进保护时被赋值）
//    volatile uint16_t final_TIMERD_cmp1;            // 实际写入TIMERD的比较值1（仅在pwm占空比计算或进保护时被赋值）
//    volatile uint16_t final_TIMERD_cmp3;            // 实际写入TIMERD的比较值3（仅在pwm占空比计算或进保护时被赋值）
//    
//} WL_RX_board_t;

///* 通信相关结构体 */
//typedef struct __attribute__((packed)) rx_info_struct {
//    
//    uint8_t  chassis_power_buffer;          // 底盘能量缓冲（在buffer_pid_ctrl中被使用）
//    uint16_t chassis_power_limit ;          // 机器人底盘功率限制上限（在power_pid_ctrl中被使用）
//    int16_t  cap_power_out_limit ;          // 电容放电功率限制，定义为负值（在cap_i_pid_ctrl中被使用）
//    uint16_t cap_power_in_limit  ;          // 电容充电功率限制，定义为正值（在cap_i_pid_ctrl中被使用）
//    
//    struct __attribute__((packed)) bit_control_struct
//    {
//        uint8_t cap_switch : 1;             // 电容开关，1为开，0为关（在check_states中的初始化状态机中被使用）
//        uint8_t turbo_mode : 1;             // 是否使用缓冲能量来充电，0为不用，1为用（在integrate_pid_ctrl中被使用）
//        uint8_t unuse      : 6;             // 暂时未使用
//    }bit_control;
//    
//} WL_RX_board_rx_info_t;

///* 通信相关结构体 */
//typedef struct __attribute__((packed)){
//    int16_t charging_power;                // 充电功率
//    uint8_t is_charging;                   // 是否在充电，1为充电，0为不充电

//    uint8_t  reserved1;
//    uint16_t reserved2;                     // 保留字段，暂时未使用
//    uint16_t reserved3;                 
//} wireless_tx_info_t;

///* Define Privated Variables Type ----------------------------------------------------------------------------------------------------------------------------------- */


///* Exported Variables Declarations ---------------------------------------------------------------------------------------------------------------------------------- */
//extern WL_RX_board_t WL_RX_board;

///* Exported Functions Declarations ---------------------------------------------------------------------------------------------------------------------------------- */
//void check_code_match(void);
//void get_master_rx_info(uint8_t *data);
//void main_control_task(void);

//#endif
