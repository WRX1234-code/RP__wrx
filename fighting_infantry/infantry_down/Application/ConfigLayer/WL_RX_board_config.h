//#ifndef _WL_RX_BOARD_CONFIG_H
//#define _WL_RX_BOARD_CONFIG_H

//#include "config.h"

///* 电容板限制，基本通用 */
//#define BAT_INDUCTION_V   (33.f)                    // 电源感应输入电压
//#define BAT_WORK_V        (12.f)                    // 电源工作输入电压
//#define BAT_V_MAX_LIMIT   (50.f)                    // 最大电源输入电压（硬性指标）
//#define CAP_I_MIN_LIMIT   (-8.f)                  // 电容组最大放电电流（硬性指标）
//#define CAP_I_MAX_LIMIT   (8.f)                   // 电容组最大充电电流（硬性指标）
//#define CAP_V_MIN_LIMIT   (2.f)                     // 电容组最小可达电压（硬性指标）
//#define CAP_V_MAX_LIMIT   (24.5f)                   // 电容组最大可达电压（硬性指标）
//#define CAP_POWER_OUT_MAX (-90.f)                  // 电容组最大放电功率（硬性指标）
//#define CAP_POWER_IN_MAX  (90.f)                   // 电容组最大充电功率（硬性指标）
//#define BOTTOM_ABILITY_PERCENTAGE_ENERGY (15.f)     // 触底电量
//#define MIN_ABILITY_PERCENTAGE_ENERGY    (30.f)     // 电量触底后，达到30%才允许放电
//#define HALF_FULL_PERCENTAGE_ENERGY      (50.f)     // 半满电电量
//#define CLOSE_FULL_PERCENTAGE_ENERGY     (90.f)     // 即将满电电量


///* 各板的个性参数 */
///* 对于第一批板，chasi和capi的方向与直觉相反，故要取反 */
///* 一号全系未投入使用 */
//#ifdef _USE_BOARD_1_x
/////* 第一批板初始校准系数 */
////#define CURRENT_BOARD_NUMBER 1_x
////#define DEVICE_UID_LOW  (uint32_t)(0x00000000)
////#define DEVICE_UID_MID  (uint32_t)(0x00000000)
////#define DEVICE_UID_MOST (uint32_t)(0x00000000)
////#define BAT_V_COEFF_A   (1.f)
////#define BAT_V_COEFF_B   (0.f)
////#define CAP_I_COEFF_A   (1.f)
////#define CAP_I_COEFF_B   (0.f)
////#define CAP_V_COEFF_A   (1.f)
////#define CAP_V_COEFF_B   (0.f)
////#define CAP_ESR         (0.3f)
////#define BAT_I_COEFF_A   (1.f)
////#define BAT_I_COEFF_B   (0.f)

///* 1-0校准系数 */
//#define CURRENT_BOARD_NUMBER 1_0
//#define DEVICE_UID_LOW  (uint32_t)(0x001B0034)//(0x001B002A)
//#define DEVICE_UID_MID  (uint32_t)(0x33435016)//(0x33435016)
//#define DEVICE_UID_MOST (uint32_t)(0x2039394B)//(0x2039394B)
//#define BAT_V_COEFF_A   (1.f)
//#define BAT_V_COEFF_B   (0.f)
//#define CAP_I_COEFF_A   (1.f)
//#define CAP_I_COEFF_B   (0.f)
//#define CAP_V_COEFF_A   (1.f)
//#define CAP_V_COEFF_B   (0.f)
//#define CAP_ESR         (0.3f)
//#define BAT_I_COEFF_A   (1.f)
//#define BAT_I_COEFF_B   (0.f)

//#endif


//#endif
