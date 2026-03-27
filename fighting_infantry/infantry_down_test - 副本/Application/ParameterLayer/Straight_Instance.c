#include "Straight_Instance.h"
#include "pid.h"

#define K_MATRIX_FIT_COEFFICIENT   \
{\
        { {-7.306359f, -183.824060f, 423.803475f, -424.029871f}, \
          {0.034843f, -21.719357f, 11.558632f, -14.732540f}, \
          {-1.616283f, -36.550640f, 106.154435f, -109.421473f}, \
          {-2.007548f, -29.882251f, 78.644252f, -80.688752f}, \
          {33.719926f, -152.002575f, 314.680956f, -252.837846f}, \
          {4.688257f, -20.071703f, 43.301116f, -36.705854f} }, \
        { {91.355967f, -242.073631f, 267.657750f, -26.868453f}, \
          {8.875316f, 6.150581f, -52.607576f, 73.000237f}, \
          {35.047958f, -159.521038f, 324.664541f, -255.007252f}, \
          {33.091682f, -156.163330f, 341.669850f, -290.273868f}, \
          {14.766092f, 814.680181f, -2373.952781f, 2457.507819f}, \
          {-1.713370f, 108.168930f, -309.871251f, 318.372258f} } \
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
