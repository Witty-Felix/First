#ifndef __SYSTEM_H
#define __SYSTEM_H

#include "stm32f10x.h"

/**************** 系统时钟配置 ****************/
/* 72MHz, HSE 8MHz -> PLL(锁相环) x9 */

/**************** SysTick配置 ****************/
#define SYSTICK_LOAD              72000                    /* 72MHz/72000=1KHz→1ms */
#define SYSTICK_PRIORITY          0                        /* SysTick中断优先级  */

/**************** NVIC分组配置 ****************/
#define NVIC_GROUP                NVIC_PriorityGroup_2     /* 2位抢占 + 2位响应 */

/* 接口函数 */
void    System_Init(void);

#endif
