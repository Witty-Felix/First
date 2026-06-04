#include "LED.h"

/*
*********************************************************************************************************
*   函 数 名: LED_Init
*   功能说明: 初始化LED
*   引    脚: LED3<PE3> | LED2<PE4>
*   形    参：无
*   返 回 值: 无
*   描    述：输出开漏，默认高电平
*********************************************************************************************************
*/
void LED_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Mode   = GPIO_Mode_Out_OD;
    gpio.GPIO_Pin    = GPIO_Pin_3 | GPIO_Pin_4;
    gpio.GPIO_Speed  = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &gpio);

    GPIO_SetBits(GPIOE, GPIO_Pin_3 | GPIO_Pin_4);
}

/**********************************************
  *  功能说明：点亮LED3,即把GPIOE 3口置低电平
  *  形    参：无
  *  返 回 值：无
***********************************************/
void LED3_ON(void)
{
    GPIO_ResetBits(GPIOE, GPIO_Pin_3);
}

/**********************************************
  *  功能说明：熄灭LED3,即把GPIOE 3口置高电平
  *  形    参：无
  *  返 回 值：无
***********************************************/
void LED3_OFF(void)
{
    GPIO_SetBits(GPIOE, GPIO_Pin_3);
}

/**********************************************
  *  功能说明：翻转LED3的亮灭
  *  形    参：无
  *  返 回 值：无
***********************************************/
void LED3_Turn(void)
{
    if(GPIO_ReadOutputDataBit(GPIOE, GPIO_Pin_3) == RESET)
        GPIO_SetBits(GPIOE, GPIO_Pin_3);
    else
        GPIO_ResetBits(GPIOE, GPIO_Pin_3);
}

/**********************************************
  *  功能说明：点亮LED2,即把GPIOE 4口置低电平
  *  形    参：无
  *  返 回 值：无
***********************************************/
void LED2_ON(void)
{
    GPIO_ResetBits(GPIOE, GPIO_Pin_4);
}

/**********************************************
  *  功能说明：熄灭LED2,即把GPIOE 4口置高电平
  *  形    参：无
  *  返 回 值：无
***********************************************/
void LED2_OFF(void)
{
    GPIO_SetBits(GPIOE, GPIO_Pin_4);
}

/**********************************************
  *  功能说明：翻转LED2的亮灭
  *  形    参：无
  *  返 回 值：无
***********************************************/
void LED2_Turn(void)
{
    if(GPIO_ReadOutputDataBit(GPIOE, GPIO_Pin_4) == RESET)
        GPIO_SetBits(GPIOE, GPIO_Pin_4);
    else
        GPIO_ResetBits(GPIOE, GPIO_Pin_4);
}
