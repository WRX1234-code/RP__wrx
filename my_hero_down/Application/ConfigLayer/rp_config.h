
#ifndef __RP_CONFIG_H
#define __RP_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include "string.h"
// ����������
#include "rp_driver_config.h"
// �豸������
#include "rp_device_config.h"
// �û�������
#include "rp_user_config.h"

/* Exported macro ------------------------------------------------------------*/

/*ѡ��IMU�����㷨ΪMahony*/
#define IMU_USE_MAHONY  1
/*ѡ��IMU�����㷨ΪEKF*/
#define IMU_USE_EKF 	0


/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/


#endif

