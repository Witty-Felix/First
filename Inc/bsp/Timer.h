#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f10x.h"

extern volatile uint8_t Timer_Poll_Flag;    /* 200ms 轮询标志 */
extern volatile uint8_t Timer_Print_Flag;   /* 5s 打印标志 */

void Timer_Init(void);

#endif
