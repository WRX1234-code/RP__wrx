/**
 * @file  device.c
 */
 
/* Includes ------------------------------------------------------------------*/
#include "device.h"
#include "judge.h"
#include "cap.h"
#include "board_protocol.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/


/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void DEVICE_Init(void)
{
	imu_sensor.init(&imu_sensor);
	rc_sensor.init(&rc_sensor);
	rm_motor_list_init();
  kt_motor_list_init();
	ht_motor_list_init();
	dm_motor_list_init();
	

	judge.init(&judge);
	cap.init(&cap);
	board.init(&board);
	
}

