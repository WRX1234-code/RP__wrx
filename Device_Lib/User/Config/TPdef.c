/* Includes ------------------------------------------------------------------*/
#include "TPdef.h"

union
{
  Interactive_Frame_t Frame;
	uint8_t InteractiveFrameBuffer[17];
} Interactive_Frame;

void Interactive_Frame_Init(void)
{
		/*
		|SOF1|data_length2|seq1|CRC81|
		|frame_header-----5----------|cmd_id 2|data n|
		|UiFrame_t(UiFrameBuffer[128])-----------|
	  */
	Interactive_Frame.InteractiveFrameBuffer[0] = 0xA5;
	Interactive_Frame.Frame.frame_header.Frame.data_length.Frame = 8;
	Interactive_Frame.Frame.frame_header.Frame.seq = 0;  
	Interactive_Frame.Frame.frame_header.Frame.CRC8 = 0;   
  Interactive_Frame.InteractiveFrameBuffer[5] = 0X06;                         
	Interactive_Frame.InteractiveFrameBuffer[6] = 0X03;
}
