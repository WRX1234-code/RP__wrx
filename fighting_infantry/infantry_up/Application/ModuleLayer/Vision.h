#ifndef  __VISION_H
#define  __VISION_H

#include <stdint.h>

typedef struct{
  uint8_t is_find_target;
	uint8_t is_keep_shooting;
	uint8_t is_enable_shooting;
  uint8_t is_find_buff;

}Vision_Info_t;


typedef enum{
  VIS_ONLINE,
	VIS_OFFLINE,

}Vision_Status_e;


typedef struct {
  uint32_t offline_cnt;
  uint32_t offline_cnt_max;
  Vision_Status_e status;
		
}Vision_State_t;


typedef struct{
  Vision_Info_t    info;
	Vision_State_t   state;


}Vision_t;


void Vision_Data_Update(void);
#endif
