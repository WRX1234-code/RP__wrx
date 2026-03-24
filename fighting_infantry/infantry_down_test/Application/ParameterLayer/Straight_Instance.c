#include "Straight_Instance.h"
#include "pid.h"

#define K_MATRIX_FIT_COEFFICIENT   \
{\
        { {-8.655832f, -153.983667f, 379.578134f, -384.853077f}, \
          {-0.268579f, -17.475458f, 15.320597f, -17.343362f}, \
          {-1.216505f, -15.790833f, 47.333383f, -49.939353f}, \
          {-1.779039f, -17.687871f, 49.542623f, -52.297735f}, \
          {32.600205f, -165.033400f, 378.391740f, -334.162079f}, \
          {4.902589f, -23.549133f, 54.386560f, -48.700365f} }, \
        { {66.845839f, -223.524167f, 371.750590f, -223.625148f}, \
          {7.357925f, -13.305139f, 13.031737f, -0.186746f}, \
          {14.802444f, -75.914306f, 172.841378f, -151.215277f}, \
          {18.862908f, -99.376077f, 235.464758f, -213.732246f}, \
          {30.089131f, 610.653757f, -1830.400434f, 1933.954952f}, \
          {1.831696f, 88.469148f, -260.875076f, 273.364733f} } \
    }
//{\
//        { {-9.822790f, -187.968507f, 432.881725f, -434.999612f}, \
//          {-0.819251f, -30.615960f, 38.625932f, -45.181095f}, \
//          {-1.680761f, -21.659807f, 62.972558f, -64.995846f}, \
//          {-2.557858f, -26.780444f, 72.193787f, -74.559571f}, \
//          {32.611081f, -151.350517f, 325.857750f, -272.830795f}, \
//          {7.000156f, -31.538486f, 68.738904f, -58.334262f} }, \
//        { {96.223941f, -257.826557f, 290.176832f, -35.284169f}, \
//          {19.706399f, -49.223247f, 73.400704f, -36.092464f}, \
//          {23.279507f, -108.857002f, 229.694661f, -187.715421f}, \
//          {31.831722f, -151.788093f, 334.275894f, -285.490870f}, \
//          {27.254237f, 717.627672f, -2094.721314f, 2172.800395f}, \
//          {3.179332f, 155.114409f, -449.828345f, 465.325222f} } \
//    }		
#define K_MATRIX_COEFFICIENT                                                        \
{\
        { -18.591714f, -1.641783f, -2.162897f, -2.873897f, 21.020018f, 3.285866f }, \
        { 48.590848f, 6.184987f, 9.417957f, 12.045904f, 66.206295f, 7.091104f } \
    }


                                                       
K_Matrix_t K_Matrix[Leg_Num] = 
{
	[R_Leg].K_coefficient=K_MATRIX_COEFFICIENT,
    [L_Leg].K_coefficient=K_MATRIX_COEFFICIENT,
	
	[R_Leg].K_coefficient_fit=K_MATRIX_FIT_COEFFICIENT,
    [L_Leg].K_coefficient_fit=K_MATRIX_FIT_COEFFICIENT,
};


X_Matrix_t X_Matrix[Leg_Num]; //×´Ì¬Á¿¾ØÕó
State_info_t Straight_Leg_info[Leg_Num];//×´Ì¬Á¿
Ex_leg_data_t Ex_leg_data[Leg_Num];//l0
u_t u[Leg_Num];//¿ØÖÆÁ¿

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
