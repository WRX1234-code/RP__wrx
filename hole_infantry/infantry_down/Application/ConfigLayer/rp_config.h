
#ifndef __RP_CONFIG_H
#define __RP_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include "string.h"
// 驱动层配置
#include "rp_driver_config.h"
// 设备层配置
#include "rp_device_config.h"
// 用户层配置
#include "rp_user_config.h"

/* Exported macro ------------------------------------------------------------*/


#define   CAP_SWITCH             1
#define   POWER_LIMIT_SWITCH     1
#define   CHASSIS_SWITCH         1
#define   SLIP_SWITCH            1












/*选择IMU解算算法为Mahony*/
#define IMU_USE_MAHONY  1
/*选择IMU解算算法为EKF*/
#define IMU_USE_EKF 	0


/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/


#endif

