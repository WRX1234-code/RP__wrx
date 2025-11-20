#ifndef __LED_H
#define __LED_H

typedef enum{
	on=0,
	down,
	blink,
}led_state;


typedef struct LED{
	GPIO_TypeDef *port;
	uint16_t pin;
	const char *color;
	uint16_t state;
	uint32_t blink_period;
}led;

extern led led_red;
extern led led_green;
extern led led_blue;


void LED_SetInit(led *led,GPIO_TypeDef *port,uint16_t pin,const char *color);
void LED_blink(led *led,uint16_t blink_period);
void LED_SetState(led *led,led_state state,uint32_t blink_period);

#endif
