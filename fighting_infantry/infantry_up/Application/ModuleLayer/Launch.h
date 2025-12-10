#ifndef  __LAUNCH_H
#define  __LAUNCH_H

#include "stdint.h"
#include "shoot_Base.h"
#include "motor.h"

#define  FRIC_NUM                              3                        
#define  FRIC_SPEED_DATA_DIRECTION_MENAGE                          

#define  FRIC_SPEED_DATA_TYPE             int16_t                  
#define  FRIC_CURRENT_DATA_TYPE           int16_t                  


#if  FRIC_NUM == 6
typedef enum{
  FRICTION_B_UP = 0,                    
	FRICTION_B_L,                         
	FRICTION_B_R,                         
	
  FRICTION_F_UP,                        
	FRICTION_F_L,                            
	FRICTION_F_R,                            
        
  FRICTION_LIST,
}Fric_Zone_e; 

#elif FRIC_NUM == 3
typedef enum{
  FRICTION_UP = 0,                      
  FRICTION_R,                           
  FRICTION_L,                           
  
	FRICTION_LIST,
}Fric_Zone_e;

#elif FRIC_NUM ==2
typedef enum{
  FRICTION_R = 0,                       
  FRICTION_L,                           

  FRICTION_LIST,
}Fric_Zone_e;

#endif


typedef struct{ 
	FRIC_SPEED_DATA_TYPE                 speed;             
  FRIC_CURRENT_DATA_TYPE               current;           
	uint8_t                              temperature;       

}Fric_Rt_Rx_Info_t;


typedef struct{
  FRIC_SPEED_DATA_TYPE          normal_speed_target;       
	FRIC_SPEED_DATA_TYPE          high_temp_speed_target;    
	uint8_t                       temp_max;                  
  FRIC_SPEED_DATA_TYPE          speed_err_max;             
  uint8_t                       temp_err_max;	             

}Fric_Base_Cfg_Rx_Info_t;


typedef struct{
	FRIC_SPEED_DATA_TYPE           speed_max;               
	FRIC_CURRENT_DATA_TYPE         current_min;             
	uint8_t                        block_time_max;                                 
  uint8_t                        menage_time_max;                                
  uint8_t                        block_time;                          
	
}Fric_Block_Cfg_Rx_Info_t;



typedef struct{
	float                           speed_max;                    
	float                           speed_min;                    
	uint8_t                         first_bullet_shield_time;     
	float                           ideal_speed_max;              
	float                           ideal_speed_min;              
	float                           ideal_death_value;            
	float                           overspeed_adjust_speed;       
	float                           high_adjust_speed;            
	float                           low_adjust_speed;             
	
	float                           high_adjust_value;            
	float                           low_adjust_value;             
	
	uint8_t                         more_cnt_max;                 
	uint8_t                         less_cnt_max;                 
 
}Fric_Self_Adapt_Cfg_Rx_Info_t ;



typedef struct{
	Fric_Base_Cfg_Rx_Info_t                  base_cfg;        
	Fric_Block_Cfg_Rx_Info_t                 block_cfg[FRICTION_LIST];       
	Fric_Self_Adapt_Cfg_Rx_Info_t             adapt_cfg;       
	
}Fric_Cfg_Rx_Info_t;

typedef struct{
	Fric_Cfg_Rx_Info_t         cfg_rx_info;
  Fric_Rt_Rx_Info_t          rt_rx_info[FRICTION_LIST];

}Fric_Rx_Info_t;

typedef struct{


}vision_Rx_Info_t ;

typedef struct{
	Fric_Rx_Info_t        fric_info;
  vision_Rx_Info_t      vision_info;

}Launch_Rx_Info_t;

typedef struct{
  FRIC_CURRENT_DATA_TYPE     output;                      
	FRIC_SPEED_DATA_TYPE       speed_target;
	
}Fric_Tar_t;


typedef struct{
	FRIC_SPEED_DATA_TYPE      speed_target;
	FRIC_SPEED_DATA_TYPE      speed_err[FRICTION_LIST];    
  int8_t                   temp_err[FRICTION_LIST];	   
	
}Fric_Check_t;

typedef struct{  
	Motor_RM_Group_t*     group;
	Fric_Tar_t            tar;
	Fric_Check_t          check;
  
}Fric_Assembly_t;

typedef struct{
  uint8_t fric_block_flag;              
	uint8_t fric_normal_speed_flag;       
	uint8_t fric_high_temp_flag;          
}Launch_Inn_Flag_t;

	
typedef struct{
	float now_speed;                       
	float shoot_freq;                      
	float muzzle_heat;                     

}Judge_Rx_Pkt_t;

typedef struct{
	uint16_t safe_cnt;
	
}Launch_Misc_t;

typedef struct{
	Fric_Assembly_t     assembly;
	Shoot_t*            base;
	Launch_Rx_Info_t    info;
	Judge_Rx_Pkt_t      judge;
  Launch_Inn_Flag_t   flag;
  Launch_Misc_t       misc;

}Launch_t;


uint8_t Fric_Block_Check(Launch_t* launch);
void Fric_State_Check(Launch_t* launch);
void Shoot_Speed_Self_Adapt(Launch_t* launch);

#endif
