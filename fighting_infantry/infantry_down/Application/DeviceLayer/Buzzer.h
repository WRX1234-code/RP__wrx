#ifndef __BUZZER_H
#define __BUZZER_H

#include "stm32h7xx_hal.h"

void Buzzer_Init(void);
void Buzzer_On(void);
void Buzzer_Off(void);
void Buzzer_ShortBeep(void);

#endif