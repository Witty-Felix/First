#include "Timer.h"

/* 全局变量 */
volatile uint8_t Timer_Poll_Flag;    /* 200ms 轮询标志 */
volatile uint8_t Timer_Print_Flag;   /* 5s 打印标志 */
static uint8_t s_5sCnt = 0;          /* 5s 打印计数 */

/*
*************************************************************************************
*   函 数 名: Timer_Init
*   功能说明: TIM2定时器初始化(200ms周期中断)
*   形    参：无
*   返 回 值: 无
*   描    述：开启TIM2时钟，配置72MHz/7200=10KHz(0.1ms)
*           ：ARR=2000，周期200ms
*           ：使能更新中断(NVIC抢占1/子优先级0)
*************************************************************************************
*/
void Timer_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_InternalClockConfig(TIM2);

    TIM_TimeBaseInitTypeDef tim;
    tim.TIM_ClockDivision      = TIM_CKD_DIV1;
    tim.TIM_CounterMode        = TIM_CounterMode_Up;
    tim.TIM_Period             = 2000 - 1;
    tim.TIM_Prescaler          = 7200 - 1;
    tim.TIM_RepetitionCounter  = 0;
    TIM_TimeBaseInit(TIM2, &tim);

    TIM_ClearFlag(TIM2, TIM_FLAG_Update);

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel                   = TIM2_IRQn;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    NVIC_Init(&nvic);

    TIM_Cmd(TIM2, ENABLE);
}

/*
*************************************************************************************
*   函 数 名: TIM2_IRQHandler
*   功能说明: TIM2中断服务函数
*   形    参：无
*   返 回 值: 无
*   描    述：置位Timer_Flag
*************************************************************************************
*/
void TIM2_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {
        Timer_Poll_Flag = 1;        /* 每200ms：轮询 */

        if(++s_5sCnt >= 25)         /* 200ms * 25 = 5s */
        {
            s_5sCnt = 0;
            Timer_Print_Flag = 1;   /* 每5s：打印 */
        }
        TIM_ClearITPendingBit(TIM2, TIM_FLAG_Update);
    }
}
