#ifndef __SYSTEM_H
#define __SYSTEM_H

#include "stm32f10x.h"

/**************** NVIC分组配置 ****************/
#define NVIC_GROUP                NVIC_PriorityGroup_2     /* 2位抢占 + 2位响应 */

/* 接口函数 */
void    System_Init(void);

#endif
