#include "Straight_Instance.h"
#include "pid.h"

#define K_MATRIX_FIT_COEFFICIENT   \
{\
        { {-5.994729f, -96.829917f, 256.877261f, -261.235940f}, \
          {-0.181770f, -9.653007f, 13.860060f, -13.786767f}, \
          {-0.368289f, -2.351922f, 7.439990f, -8.123954f}, \
          {-0.774366f, -4.356430f, 13.317739f, -14.557527f}, \
          {12.525479f, -71.622603f, 178.804290f, -168.403047f}, \
          {0.896626f, -4.189000f, 10.000279f, -9.192825f} }, \
        { {40.744093f, -161.864428f, 318.373511f, -232.980200f}, \
          {4.750764f, -20.444212f, 41.563508f, -33.730678f}, \
          {3.098384f, -19.137157f, 48.432478f, -45.671470f}, \
          {6.286572f, -39.383214f, 101.145957f, -96.625017f}, \
          {15.723028f, 230.963814f, -718.350923f, 778.181407f}, \
          {0.437962f, 14.859938f, -44.011699f, 46.322355f} } \
    }


		
#define K_MATRIX_COEFFICIENT                                                        \
{\
        { -12.169611f, -0.908689f, -0.507658f, -1.038903f, 8.963520f, 0.655689f }, \
        { 27.032185f, 3.123875f, 1.758723f, 3.579573f, 37.584384f, 1.568318f } \
    }


//{\
//        { -13.027295f, -0.911927f, -0.537415f, -1.229951f, 12.632588f, 0.935461f }, \
//        { 34.078283f, 3.243489f, 1.554691f, 3.557717f, 41.738195f, 1.636098f } \
//    }   //有云台的定腿长K矩阵

                                                        \
		
	
K_Matrix_t K_Matrix[Leg_Num] = 
{
	[R_Leg].K_coefficient=K_MATRIX_COEFFICIENT,
    [L_Leg].K_coefficient=K_MATRIX_COEFFICIENT,
	
	[R_Leg].K_coefficient_fit=K_MATRIX_FIT_COEFFICIENT,
    [L_Leg].K_coefficient_fit=K_MATRIX_FIT_COEFFICIENT,
};


X_Matrix_t X_Matrix[Leg_Num]; //状态量矩阵
State_info_t Straight_Leg_info[Leg_Num];//状态量
Ex_leg_data_t Ex_leg_data[Leg_Num];//l0
u_t u[Leg_Num];//控制量

Straight_Leg_t Straight_Leg[Leg_Num] = {
	[R_Leg] =
	{
		.info = &Straight_Leg_info[R_Leg],
	
		.K_info = &K_Matrix[R_Leg],
	
		.X_info = &X_Matrix[R_Leg],
	
		.Ex_leg_data=&Ex_leg_data[R_Leg],
		
		.u=&u[R_Leg],
		
		.init = Straight_Leg_Init,
	},
	[L_Leg] =
	{
		.info = &Straight_Leg_info[L_Leg],
	
		.K_info = &K_Matrix[L_Leg],
	
		.X_info = &X_Matrix[L_Leg],
	
		.Ex_leg_data=&Ex_leg_data[L_Leg],
		
		.u=&u[L_Leg],
		
		.init = Straight_Leg_Init,
	}
	
	
};
