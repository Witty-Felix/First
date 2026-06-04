#include "Motor.h"
#include "LimSwc.h"

/* 全局变量 */
volatile uint8_t  Move_Stop;
volatile uint8_t  Move_End;
volatile uint8_t  Motor_Flag;
volatile uint8_t  hit_cnt;
volatile uint8_t  Lock_Flag;
volatile uint8_t  Cnt_Stop;
volatile uint16_t pulse_per_round;
Motor_t  Motor;
RunState g_RunState = HOME_INIT;
volatile uint32_t Hit_ms = 0;
volatile uint32_t Stop_ms = 0;

/*
*************************************************************************************
*   函 数 名: Motor_DirEn_Init
*   功能说明: 电机方向及使能引脚初始化
*   引    脚: DIR<PE12> | EN<PB11>
*   形    参：无
*   返 回 值: 无
*   描    述：配置为推挽输出，默认低电平
*************************************************************************************
*/
void Motor_DirEn_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Mode   = GPIO_Mode_Out_PP;
    gpio.GPIO_Pin    = GPIO_Pin_12;
    gpio.GPIO_Speed  = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &gpio);

    gpio.GPIO_Pin    = GPIO_Pin_11;
    GPIO_Init(GPIOB, &gpio);
}

/*
*************************************************************************************
*   函 数 名: Motor_Dir
*   功能说明: 设置电机方向
*   形    参：val - FORWARD(正转) / BACKWARD(反转)
*   返 回 值: 无
*************************************************************************************
*/
void Motor_Dir(Motor_Dir_e val)
{
    if(val)
        GPIO_SetBits(GPIOE, GPIO_Pin_12);
    else
        GPIO_ResetBits(GPIOE, GPIO_Pin_12);
}

/*
*************************************************************************************
*   函 数 名: Motor_EN
*   功能说明: 使能/禁能电机
*   形    参：val - ENA(使能) / DIS(禁能)
*   返 回 值: 无
*************************************************************************************
*/
void Motor_EN(Motor_En_e val)
{
    if(!val)
        GPIO_ResetBits(GPIOB, GPIO_Pin_11);
    else
        GPIO_SetBits(GPIOB, GPIO_Pin_11);
}

/*
*************************************************************************************
*   函 数 名: Pulse_Cnt_Exti
*   功能说明: 脉冲计数-EXTI中断初始化
*   引    脚: PA0(TIM5_CH1 PWM输出引脚)
*   形    参：无
*   返 回 值: 无
*   描    述：将PA0映射至EXTI_Line0，上升沿触发中断
*           ：每输出一个PWM脉冲，EXTI0中断自增Pulse_Count
*           ：NVIC优先级(抢占1/子优先级1)
*************************************************************************************
*/
static void Pulse_Cnt_Exti(void)
{
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

    EXTI_InitTypeDef  exti;
    exti.EXTI_Line    = EXTI_Line0;
    exti.EXTI_LineCmd = ENABLE;
    exti.EXTI_Mode    = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_Init(&exti);

    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel                   = EXTI0_IRQn;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    NVIC_Init(&nvic);
}

/*
*************************************************************************************
*   函 数 名: Motor_PWM_Init
*   功能说明: TIM5 PWM初始化(PA0)
*   形    参：arr - 自动重装值 | psc - 预分频值
*   返 回 值: 无
*   描    述：配置为PWM1模式，极性高，使能预装载
*           ：初始化完不启动，需调用Motor_Pulse(ENABLE)
*************************************************************************************
*/
void Motor_PWM_Init(uint16_t arr, uint16_t psc)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Mode   = GPIO_Mode_AF_PP;
    gpio.GPIO_Pin    = GPIO_Pin_0;
    gpio.GPIO_Speed  = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    TIM_InternalClockConfig(TIM5);

    TIM_TimeBaseInitTypeDef tim;
    tim.TIM_ClockDivision      = TIM_CKD_DIV1;
    tim.TIM_CounterMode        = TIM_CounterMode_Up;
    tim.TIM_Period             = arr - 1;
    tim.TIM_Prescaler          = psc - 1;
    tim.TIM_RepetitionCounter  = 0;
    TIM_TimeBaseInit(TIM5, &tim);

    TIM_OCInitTypeDef oc;
    TIM_OCStructInit(&oc);
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse       = 0;
    TIM_OC1Init(TIM5, &oc);

    TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM5, ENABLE);

    TIM_Cmd(TIM5, DISABLE);

    Pulse_Cnt_Exti();
}

/*
*************************************************************************************
*   函 数 名: Motor_SetARR
*   功能说明: 设置PWM频率
*   形    参：speed - 目标速度(脉冲/秒)
*   返 回 值: 无
*   描    述：ARR = 7200000 / speed - 1
*************************************************************************************
*/
void Motor_SetARR(uint16_t speed)
{
    TIM_SetAutoreload(TIM5, 7200000 / speed - 1);
}

/*
*************************************************************************************
*   函 数 名: Motor_SetCRR
*   功能说明: 设置PWM占空比(50%)
*   形    参：speed - 目标速度(脉冲/秒)
*   返 回 值: 无
*   描    述：CCR = 3600000 / speed
*************************************************************************************
*/
void Motor_SetCRR(uint16_t speed)
{
    TIM_SetCompare1(TIM5, 3600000 / speed);
}

/*
*************************************************************************************
*   函 数 名: Motor_Speed
*   功能说明: 设置电机速度
*   形    参：Speed - 目标速度 200~2000(脉冲/秒)
*   返 回 值: 无
*************************************************************************************
*/
void Motor_Speed(uint16_t Speed)
{
    Motor_SetARR(Speed);
    Motor_SetCRR(Speed);
}

/*
*************************************************************************************
*   函 数 名: Motor_Pulse
*   功能说明: 电机脉冲输出开关
*   形    参：val - ENABLE(开启) / DISABLE(关闭)
*   返 回 值: 无
*************************************************************************************
*/
void Motor_Pulse(FunctionalState val)
{
    TIM_Cmd(TIM5, val);
}

/*
*************************************************************************************
*   函 数 名: Motor_Init
*   功能说明: 电机参数初始化
*   形    参：Motor - 电机结构体指针
*   返 回 值: 无
*   描    述：读取限位开关确定初始方向，初始化PWM
*************************************************************************************
*/
void Motor_Init(Motor_t* Motor)
{
    Motor->Motor_Dir = FORWARD;
    Motor->Motor_En    = ENA;
    Motor->Pulse_Count = 0;
    Motor->Pulse_Max   = 64000;
    Motor->Speed       = 500;

    Motor_DirEn_Init();
    Motor_PWM_Init(7200, 10);
}

/*
*************************************************************************************
*   函 数 名: Motor_Start
*   功能说明: 电机启动
*   形    参：Motor - 电机结构体指针
*   返 回 值: 无
*   描    述：设置方向、使能、速度，使能脉冲输出
*************************************************************************************
*/
void Motor_Start(Motor_t* Motor)
{
    Motor_Dir(Motor->Motor_Dir);
    Motor_EN(Motor->Motor_En);
    Motor_Speed(Motor->Speed);
    Move_Stop = 0;
    Motor_Pulse(ENABLE);
}

/*
*************************************************************************************
*   函 数 名: Motor_Stop
*   功能说明: 电机停止
*   形    参：无
*   返 回 值: 无
*************************************************************************************
*/
void Motor_Stop(void)
{
    Motor_Pulse(DISABLE);
    Move_Stop = 1;
}

/*
*************************************************************************************
*   函 数 名: Motor_Resume
*   功能说明: 电机恢复脉冲输出
*   形    参：无
*   返 回 值: 无
*************************************************************************************
*/
void Motor_Resume(void)
{
    Motor_Pulse(ENABLE);
    Move_Stop = 0;
}

/*
*************************************************************************************
*   函 数 名: Motor_Change_Dir
*   功能说明: 电机方向切换
*   形    参：Motor - 电机结构体指针
*   返 回 值: 无
*   描    述：FORWARD ↔ BACKWARD
*************************************************************************************
*/
void Motor_Change_Dir(Motor_t* Motor)
{
    if(Motor->Motor_Dir == FORWARD)
        Motor->Motor_Dir = BACKWARD;
    else
        Motor->Motor_Dir = FORWARD;
}

/*
*************************************************************************************
*   函 数 名: Motor_HandleLockState
*   功能说明: 碰撞自学习 & Lock后定点停止处理
*   形    参：Motor - 电机结构体指针
*   返 回 值: 无
*   描    述：hit_cnt==2：记录脉冲数，启用Lock模式
*           ：Lock后脉冲到达1/4或3/4位置时停止2s再启动
*************************************************************************************
*/
static void Motor_HandleLockState(Motor_t* Motor)
{
    /*
     * 撞击两次后（起点→右→左），此时的Pulse_Count即为完整单程脉冲数
     * Lock后EXTI中断按pulse_per_round归零，实现定行程往返
     */
    if(hit_cnt == 3)
    {
        Lock_Flag = 1;
        pulse_per_round = Motor->Pulse_Count;
        printf("Pulse_per_round = %d", pulse_per_round);
        Motor->Pulse_Count = 0;
    }
    if(Lock_Flag == 1)
    {
        if((4 * (Motor->Pulse_Count)) >= pulse_per_round || (4 * (Motor->Pulse_Count)) >= (3 * pulse_per_round))
        {
            Motor_Stop();
            Stop_ms = SysTick_ms;
            if((SysTick_ms - Stop_ms) > 2000)
            {
                Motor_Start(Motor);
            }
        }
    }
}

/*
*************************************************************************************
*   函 数 名: Motor_Check_State
*   功能说明: 电机运行状态检测
*   形    参：Motor - 电机结构体指针
*   返 回 值: 无
*   描    述：根据限位开关状态切换运行模式
*           ：FWD→限位变向→BKD→限位停止
*************************************************************************************
*/
void Motor_Check_State(Motor_t* Motor)
{
    uint8_t hit_right = LimSwc_ReadRight();
    uint8_t hit_left = LimSwc_ReadLeft();

    switch(g_RunState)
    {
        case HOME_INIT:
            if(!hit_right)
            {
                Motor->Motor_Dir = BACKWARD;
                Motor_Start(Motor);
            }
            g_RunState = HOME_CHECK;
            break;
        case HOME_CHECK:
            if(hit_right)
            {
                Motor_Stop();
                g_RunState = WAIT_START;
            }
            break;
        case WAIT_START:
            if(Get_USART3_Data())
            {
                if(strncmp("START",(char*)USART3_RxBuffer, 5) == 0)
                {
                    Motor->Motor_Dir = FORWARD;
                    Motor->Pulse_Count = 0;
                    Motor_Start(Motor);
                    Move_Stop = 0;
                    g_RunState = WAIT_HIT;
                }
                flashUART3();
            }
            break;
        case WAIT_HIT:
            if(hit_left)
            {
                if(Cnt_Stop == 0)
                {
                    hit_cnt++;
                    Cnt_Stop = 1;
                    Hit_ms = SysTick_ms;
                }
                Motor->Motor_Dir = BACKWARD;
                Motor_Start(Motor);
            }
            else if(hit_right)
            {
                if(Cnt_Stop == 0)
                {
                    hit_cnt++;
                    Cnt_Stop = 1;
                    Hit_ms = SysTick_ms;
                }
                Motor->Motor_Dir = FORWARD;
                Motor_Start(Motor);
            }
            if((SysTick_ms - Hit_ms) > 300)
            {
                Cnt_Stop = 0;
            }
            Motor_HandleLockState(Motor);
            break;
            
            
            
    }
}


/*
*************************************************************************************
*   函 数 名: Motor_CheckPulseOverflow
*   功能说明: 检查脉冲计数是否溢出，溢出时归零
*   形    参：无
*   返 回 值: 无
*   描    述：Lock模式下超过pulse_per_round则归零
*           ：超过Pulse_Max则归零
*************************************************************************************
*/
static void Motor_CheckPulseOverflow(void)
{
    if(Lock_Flag == 1)
    {
        if(Motor.Pulse_Count >= pulse_per_round)
        {
            Motor.Pulse_Count = 0;
        }
    }
    if(Motor.Pulse_Count >= Motor.Pulse_Max)
    {
        Motor.Pulse_Count = 0;
    }
}

/*
*************************************************************************************
*   函 数 名: EXTI0_IRQHandler
*   功能说明: EXTI0中断服务函数
*   形    参：无
*   返 回 值: 无
*   描    述：PA0(TIM5_PWM输出)上升沿触发，每步递增Pulse_Count
*           ：调用Motor_CheckPulseOverflow()处理溢出归零
*************************************************************************************
*/
void EXTI0_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line0))
    {
        Motor.Pulse_Count++;
        Motor_CheckPulseOverflow();
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}
