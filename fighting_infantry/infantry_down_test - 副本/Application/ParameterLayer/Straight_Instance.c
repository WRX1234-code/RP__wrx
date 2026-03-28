#include "Straight_Instance.h"
#include "pid.h"

#define K_MATRIX_FIT_COEFFICIENT   \
{\
        { {-6.818416f, -180.651338f, 468.970591f, -488.346826f}, \
          {-0.084056f, -18.861908f, 18.052545f, -19.531385f}, \
          {-0.820853f, -19.195157f, 58.366618f, -62.572004f}, \
          {-1.412419f, -21.767300f, 62.936666f, -67.520542f}, \
          {33.361522f, -157.941754f, 336.539561f, -276.015084f}, \
          {4.940788f, -22.679951f, 50.356337f, -43.675015f} }, \
        { {84.274900f, -249.653014f, 287.746641f, -24.046340f}, \
          {8.410475f, -6.495890f, -20.854452f, 44.511916f}, \
          {17.319336f, -82.275148f, 171.554567f, -136.576517f}, \
          {23.367668f, -118.423166f, 270.283185f, -237.481684f}, \
          {15.433279f, 857.677492f, -2621.706458f, 2826.667727f}, \
          {-0.597284f, 119.427429f, -359.968586f, 385.643633f} } \
    }


//{\
//        { {-8.655832f, -153.983667f, 379.578134f, -384.853077f}, \
//          {-0.268579f, -17.475458f, 15.320597f, -17.343362f}, \
//          {-1.216505f, -15.790833f, 47.333383f, -49.939353f}, \
//          {-1.779039f, -17.687871f, 49.542623f, -52.297735f}, \
//          {32.600205f, -165.033400f, 378.391740f, -334.162079f}, \
//          {4.902589f, -23.549133f, 54.386560f, -48.700365f} }, \
//        { {66.845839f, -223.524167f, 371.750590f, -223.625148f}, \
//          {7.357925f, -13.305139f, 13.031737f, -0.186746f}, \
//          {14.802444f, -75.914306f, 172.841378f, -151.215277f}, \
//          {18.862908f, -99.376077f, 235.464758f, -213.732246f}, \
//          {30.089131f, 610.653757f, -1830.400434f, 1933.954952f}, \
//          {1.831696f, 88.469148f, -260.875076f, 273.364733f} } \
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
