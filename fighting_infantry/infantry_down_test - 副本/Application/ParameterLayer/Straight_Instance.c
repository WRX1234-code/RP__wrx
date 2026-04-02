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
//        { {-6.716984f, -182.163226f, 464.932732f, -483.034943f}, \
//          {-0.067768f, -18.990524f, 15.451092f, -17.101386f}, \
//          {-0.726656f, -17.155304f, 52.122335f, -55.881107f}, \
//          {-1.367953f, -22.621681f, 65.270269f, -69.906434f}, \
//          {33.352795f, -157.003578f, 336.101846f, -277.118999f}, \
//          {4.943507f, -22.521620f, 50.107298f, -43.537436f} }, \
//        { {84.064780f, -238.429450f, 264.912793f, -5.968851f}, \
//          {8.306640f, -3.788246f, -24.510813f, 46.822201f}, \
//          {15.489882f, -73.223651f, 153.577108f, -123.112574f}, \
//          {23.562050f, -117.292650f, 265.786282f, -231.894557f}, \
//          {15.027993f, 857.042794f, -2617.353277f, 2821.813829f}, \
//          {-0.701196f, 119.574869f, -359.901856f, 385.384041f} } \
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
