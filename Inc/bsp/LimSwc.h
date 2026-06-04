#ifndef __LIMSWC_H
#define __LIMSWC_H

#include "stm32f10x.h"
#include "Motor.h"

void    LimSwc_Init(void);
uint8_t LimSwc_ReadRight(void);
uint8_t LimSwc_ReadLeft(void);

#endif
