#include "LimSwc.h"

/*
*************************************************************************************
*   函 数 名: LimSwc_Init
*   功能说明: 初始化限位开关
*   引    脚: RIGHT<PC13> | LEFT<PC14>
*   形    参：无
*   返 回 值: 无
*   描    述：开启GPIO及AFIO时钟，配置为上拉输入模式
*           ：映射EXTI中断线(PC13→ET_Line13, PC14→ET_Line14)
*           ：设置下降沿触发中断，使能NVIC(抢占1/子优先级1)
*************************************************************************************
*/
void LimSwc_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Mode   = GPIO_Mode_IPU;
    gpio.GPIO_Pin    = GPIO_Pin_13 | GPIO_Pin_14;
    gpio.GPIO_Speed  = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpio);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource13);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource14);

    EXTI_InitTypeDef  exti;
    exti.EXTI_Line    = EXTI_Line13 | EXTI_Line14;
    exti.EXTI_LineCmd = ENABLE;
    exti.EXTI_Mode    = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_Init(&exti);

    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel                   = EXTI15_10_IRQn;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 1;
    NVIC_Init(&nvic);
}

void EXTI15_10_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line13) == SET)
    {
        EXTI_ClearITPendingBit(EXTI_Line13);
    }
    if(EXTI_GetITStatus(EXTI_Line14) == SET)
    {
        EXTI_ClearITPendingBit(EXTI_Line14);
    }
}

/*
*************************************************************************************
*   函 数 名: LimSwc_ReadRight
*   功能说明: 读取右限位开关状态
*   形    参：无
*   返 回 值: 1-触发(低电平) / 0-未触发
*************************************************************************************
*/
uint8_t LimSwc_ReadRight(void)
{
    return (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == RESET);
}

/*
*************************************************************************************
*   函 数 名: LimSwc_ReadLeft
*   功能说明: 读取左限位开关状态
*   形    参：无
*   返 回 值: 1-触发(低电平) / 0-未触发
*************************************************************************************
*/
uint8_t LimSwc_ReadLeft(void)
{
    return (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14) == RESET);
}
