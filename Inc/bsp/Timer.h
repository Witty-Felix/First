#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f10x.h"

extern volatile uint8_t Timer_Poll_Flag;    
extern volatile uint8_t Timer_Print_Flag;  
extern volatile uint32_t Timer_ms;

void Timer_Init(void);

#endif
