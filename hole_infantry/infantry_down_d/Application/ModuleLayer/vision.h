#ifndef __VISION_H
#define __VISION_H


typedef enum{
	V_NORMAL,
	V_AUTO,
	V_S_BUFF,
	V_B_BUFF,
  V_OUTPOST,
	V_HERO,
	VISION_CNT,

}Vision_Mode_e;


typedef struct Vision_Struct_t{
	Vision_Mode_e         mode;
	
	void (*init)(struct Vision_Struct_t* vision);
	void (*work)(struct Vision_Struct_t* vision);
	
}Vision_t;


extern Vision_t vision;



#endif 
