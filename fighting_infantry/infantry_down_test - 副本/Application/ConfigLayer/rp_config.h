/**
 * @file        rp_config.c
 * @author      RobotPilots
 * @Version     v1.1
 * @brief       RobotPilots Robots' Configuration.
 * @update
 *              v1.0(9-September-2020)
 *              v1.1(7-November-2021)
 *                  1.优化设备类信息与结构体的变量定义，增加volatile/const关键字
 *                  //2.将rp_config.h分成driver_config.h, device_config.h, user_config.h三个头文件    
 */
#ifndef __RP_CONFIG_H
#define __RP_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "stdbool.h"
#include "string.h"
// 驱动层配置
#include "rp_driver_config.h"
// 设备层配置
#include "rp_device_config.h"
// 用户层配置
#include "rp_user_config.h"

/* Exported macro ------------------------------------------------------------*/

/*---------------------------以下开关，0为关，1为开----------------------------*/

/*---------------比赛要开---------------*/

/*超电开关*/
#define CAP_SWITCH		               1
/*裁判系统开关*/
#define JUDGE_SWITCH 	               1
/*底盘开关*/
#define CHASSIS_SWITCH               1
/*K矩阵拟合开关*/
#define K_Fitting_SWITCH             1
/*自救开关*/
#define RESCUE_SWITCH                1
/*跳跃伸腿缓冲开关*/
#define PRE_LANDING_SWITCH           1
/*功率限制开关*/
#define POWER_LIMIT_SWITCH           1
/*氮气弹簧前馈开关*/
#define SPRING_SWITCH                1    
/*机械用测试开关*/
#define FAKE_TEST_SWITCH             1
/*拨盘复位控制开关*/
#define DIAL_RESET_SWITCH            1


/*选择IMU解算算法为Mahony*/
#define IMU_USE_MAHONY               0
/*选择IMU解算算法为EKF*/
#define IMU_USE_EKF 	               1
/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
//以下为汇编函数
void WFI_SET(void);		//执行WFI指令
void INTX_DISABLE(void);//关闭所有中断
void INTX_ENABLE(void);	//开启所有中断
void MSR_MSP(uint32_t addr);	//设置堆栈地址 

#endif
