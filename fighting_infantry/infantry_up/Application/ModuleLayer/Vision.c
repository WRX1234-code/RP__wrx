#include "vision.h"
#include "vision_protocol.h"
#include "Board_protocol.h"


void Vision_Data_Update(void)
{
	vision_tx_frame.x_v = C_Board_Rx_Info.v_x;
	vision_tx_frame.y_v = C_Board_Rx_Info.v_y;
	
	vision_tx_frame.blood[0] = C_Board_Rx_Info.blood_0;
	vision_tx_frame.blood[1] = C_Board_Rx_Info.blood_1;
	vision_tx_frame.blood[2] = C_Board_Rx_Info.blood_2;
	vision_tx_frame.blood[3] = C_Board_Rx_Info.blood_3;
	vision_tx_frame.blood[4] = C_Board_Rx_Info.blood_4;
	vision_tx_frame.blood[5] = C_Board_Rx_Info.blood_5;
	vision_tx_frame.blood[6] = C_Board_Rx_Info.blood_6;
	vision_tx_frame.blood[7] = C_Board_Rx_Info.blood_7;
	
	
	vision_tx_frame.own_color = C_Board_Rx_Info.my_color;
	
	if(C_Board_Rx_Info.is_video_open == 1)
	{
		vision_tx_frame.game_start = 1;
	}
	else{
	  vision_tx_frame.game_start = 0;
	} 

	
}