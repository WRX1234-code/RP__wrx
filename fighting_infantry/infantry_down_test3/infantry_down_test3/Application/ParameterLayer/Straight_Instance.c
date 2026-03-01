#include "Straight_Instance.h"
#include "pid.h"

#define K_MATRIX_FIT_COEFFICIENT   \
{\
        { {-9.825001f, -188.311855f, 392.725602f, -397.773685f}, \
          {-0.355843f, -26.108212f, 9.307812f, -18.611235f}, \
          {-3.719956f, -48.785594f, 142.576078f, -148.004464f}, \
          {-3.574138f, -33.894039f, 83.672136f, -85.960903f}, \
          {32.821915f, -153.288791f, 338.806290f, -291.900036f}, \
          {5.192100f, -22.937314f, 52.302481f, -46.495832f} }, \
        { {82.194016f, -203.995438f, 253.808855f, -72.315766f}, \
          {10.548508f, -3.030692f, -18.118844f, 35.253781f}, \
          {45.859047f, -216.782373f, 470.482442f, -397.402503f}, \
          {36.067933f, -171.632117f, 391.494546f, -347.459082f}, \
          {24.530306f, 627.683296f, -1838.996029f, 1916.352950f}, \
          {-0.172440f, 99.349216f, -287.179781f, 297.676077f} } \
    }
		
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
