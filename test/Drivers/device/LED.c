#include "stm32f4xx.h"                  // Device header
#include "gpio.h"

typedef enum{
	on=0,
	down,
	blink,
}led_state;


typedef struct LED{
	GPIO_TypeDef *port;
	uint16_t pin;
	const char *color;
	led_state state;
	uint32_t blink_period;
}led;

led led_blue;     //PH10
led led_green;    //PH11
led led_red;      //PH12

void LED_SetInit(led *led,GPIO_TypeDef *port,uint16_t pin,const char *color)
{
	led->port=port;
	led->pin=pin;
	led->color=color;
  
}

void LED_blink(led *led,uint32_t blink_period)
{
	led->blink_period=blink_period;
	HAL_GPIO_TogglePin(led->port,led->pin);
	uint32_t time_start=HAL_GetTick();;
	while(HAL_GetTick()-time_start<blink_period/2){}
	HAL_GPIO_TogglePin(led->port,led->pin);
	uint32_t time_last=HAL_GetTick();;
	while(HAL_GetTick()-time_last<blink_period/2){}
}



void LED_SetState(led *led,led_state state,uint16_t blink_period)
{
	led->state=state;
	if(state==on)
	{
		HAL_GPIO_WritePin(led->port,led->pin,GPIO_PIN_SET);
	}
	else if(state==down)
	{
		HAL_GPIO_WritePin(led->port,led->pin,GPIO_PIN_RESET);
	}
	else if(state==blink)
	{
		LED_blink(led,blink_period);

	}
}





