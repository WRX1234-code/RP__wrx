#ifndef  __ROBOT_H
#define  __ROBOT_H


typedef enum{
	RC_CU = 0,
	KEY_CU,


}Robot_CU_e;


typedef enum{
	MEC,
	GYRO,
	S_GYRO,
	
}Robot_Base_Mode_e;


typedef enum{
	NO_ADV_MODE,
	H_S_S_GYRO,
  SELF_AIM,
	MELEE,
	SUSPEND,
}Robot_Adv_Mode_e;


typedef enum{
	ONLINE,
	OFFLINE,
  

}Robot_State_e;

typedef enum{
	RISING_EDGE = 0,
	FALLING_EDGE,
	HIGH_LEVEL,
	LOW_LEVEL,

}Robot_Elec_Level_e;


typedef struct{
	Robot_CU_e            CU ;
	Robot_Base_Mode_e     base_mode;
	Robot_Adv_Mode_e      adv_mode;
	Robot_State_e         state;
  Robot_Elec_Level_e    elec_level;
	
}Robot_t;

extern Robot_t robot;

#endif
