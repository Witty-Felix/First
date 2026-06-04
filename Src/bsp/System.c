#include "System.h"

/*
*************************************************************************************
*   函 数 名: SysTick_Init
*   功能说明: SysTick定时器初始化(1ms中断)
*   形    参：无
*   返 回 值: 无
*   描    述：配置72MHz/72000=1KHz(1ms)，设置中断优先级0
*************************************************************************************
*/
static void SysTick_Init(void)
{
    SysTick_Config(SYSTICK_LOAD);
    NVIC_SetPriority(SysTick_IRQn, SYSTICK_PRIORITY);
}

/*
*************************************************************************************
*   函 数 名: System_Init
*   功能说明: 系统初始化
*   形    参：无
*   返 回 值: 无
*   描    述：配置NVIC优先级分组，初始化SysTick定时器
*************************************************************************************
*/
void System_Init(void)
{
    /* NVIC分组：2位抢占 + 2位响应 */
    NVIC_PriorityGroupConfig(NVIC_GROUP);

    /* SysTick定时器初始化 */
    SysTick_Init();
}
