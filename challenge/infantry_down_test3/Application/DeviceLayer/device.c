/**
 * @file  device.c
 */
 
/* Includes ------------------------------------------------------------------*/
#include "device.h"


/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
dev_list_t dev_list = {
	.rc_sen     = &rc_sensor,
};

/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void DEVICE_Init(void)
{

	dev_list.rc_sen->init(dev_list.rc_sen);
	rc_sensor.init(&rc_sensor);
	imu_sensor.init(&imu_sensor);
	/*只能放在imu初始化后面 begin*/
	Chassis.Init(&Chassis);
	Balance.init(&Balance);
	/*只能放在imu初始化后面 end*/
	/*软件层初始化*/
	Sd_Group.group_init(&Sd_Group);
	Wheel_Group.group_init(&Wheel_Group);
	Yaw_Motor_Init();
	Dial_Motor_Init();
	CAN1_Group.group_init(&CAN1_Group);
	Cmd_Init();
	
	/*裁判系统初始化*/
	Judge_Init(&judge);
	cap.init(&cap);
	
	
}
