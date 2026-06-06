#include "Motor.h"
#include "LimSwc.h"
#include "Delay.h"
#include "System.h"

/* 全局变量 */
uint8_t          Move_Stop;
uint8_t          Motor_Flag;
volatile Motor_t Motor;
RunState         g_RunState = HOME_INIT;
static uint32_t  Stop_ms;
volatile uint8_t Obstacle_Flag;
static uint8_t   s_RecoverCnt;     /* 安全读数计数 */
static uint32_t  s_ObstacleStart;  /* 障碍物触发时刻(Timer_ms)，用于暂停时序 */

#define OBST_MAX_DIST        30   /* 急停距离阈值(mm)，小于此值触发急停 */
#define START_OBST_MIN_DIST  50  /* 启动障碍物最小距离 */
#define OBST_RECOVER_CNT     5    /* 需连续5次安全读数才解除急停 */

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
static void Motor_DirEn_Init(void)
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
*   函 数 名: Motor_PWM_Init
*   功能说明: TIM5 PWM初始化(PA0)
*   形    参：arr - 自动重装值 | psc - 预分频值
*   返 回 值: 无
*   描    述：配置为PWM1模式，极性高，使能预装载
*           ：初始化完不启动，需调用Motor_Pulse(ENABLE)
*************************************************************************************
*/
static void Motor_PWM_Init(uint16_t arr, uint16_t psc)
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

    /* 开启TIM5更新中断：每发一个PWM脉冲，Pulse_Count++ */
    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);

    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel                   = TIM5_IRQn;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 2;
    NVIC_Init(&nvic);
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
void Motor_Init(volatile Motor_t* Motor)
{
    if(LimSwc_ReadLeft())
        Motor->Motor_Dir = BACKWARD;
    else
        Motor->Motor_Dir = FORWARD;

    Motor->Motor_En    = ENA;
    Motor->Pulse_Count = 0;
    Motor->Pulse_Max   = 5650;
    Motor->Speed       = 800;

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
void Motor_Start(volatile Motor_t* Motor)
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
void Motor_Change_Dir(volatile Motor_t* Motor)
{
    if(Motor->Motor_Dir == FORWARD)
        Motor->Motor_Dir = BACKWARD;
    else
        Motor->Motor_Dir = FORWARD;
}

/*
*************************************************************************************
*   函 数 名: Motor_Check_State
*   功能说明: 电机运行状态机
*   形    参：Motor - 电机结构体指针
*   返 回 值: 无
*   描    述：上电寻原点 → 等待启动命令 → 定点运动(到中点停2秒)
*************************************************************************************
*/
void Motor_Check_State(volatile Motor_t* Motor)
{
    uint8_t hit_right = LimSwc_ReadRight();
    uint8_t hit_left = LimSwc_ReadLeft();

    /*
     *  ★ 有障碍物 → 禁止发数据、禁止运动，直接返回
     *  （MID_STOP / MID_STOP2 / MID_STOP3 / BACK_STOP
     *  中调用的
     *  RS485_U1_Data_Print() 也不会被执行）
    */
    if(RADAR_RX_DIST <= OBST_MAX_DIST && RADAR_RX_DIST != 0)
    {
        if(!Obstacle_Flag)
        {
            Motor_Stop();
            Obstacle_Flag    = 1;
            s_RecoverCnt     = 0;
            s_ObstacleStart  = Timer_ms;      /* 记录急停时刻，用于暂停时序 */
            LED_Show_Red();
            printf("DIST = %d\n", RADAR_RX_DIST);
        }
    }

    /* 恢复检测 */
    if(Obstacle_Flag == 1)
    {
        if(RADAR_RX_DIST >= START_OBST_MIN_DIST)  
        {
            s_RecoverCnt++;
        }
        else
        {
            s_RecoverCnt = 0;
        }

        if(s_RecoverCnt >= OBST_RECOVER_CNT)
        {
            /* 将阻塞时长补偿到 Stop_ms，实现"暂停时序"效果 */
            Stop_ms += (Timer_ms - s_ObstacleStart);

            /* 仅在电机正在运动中才恢复脉冲输出 */
            if(g_RunState == HOME_CHECK || g_RunState == WAIT_HIT ||
               g_RunState == GO_END   || g_RunState == BACK_MID ||
               g_RunState == GO_BACK)
            {
                Motor_Resume();
                LED_Show_Green();
            }
            else
            {
                LED_Show_Yellow();   /* 暂停态恢复 → 黄灯(继续采集) */
            }

            Obstacle_Flag = 0;
            s_RecoverCnt  = 0;
            printf("DIST = %d\n", RADAR_RX_DIST);
        }
    }

    if(Obstacle_Flag == 1)
        return;

    switch(g_RunState)
    {
        /*=========== 上电寻原点 ===========*/
        case HOME_INIT:
            if(!hit_right)
            {
                Motor->Motor_Dir = BACKWARD;
                Motor_Start(Motor);
            }
            g_RunState = HOME_CHECK;
            break;

        case HOME_CHECK:
            if(hit_right)                  /* 碰到右限位→回到原点了 */
            {
                Motor_Stop();
                Motor->Pulse_Count = 0;    /* 位置归零 */
                g_RunState = WAIT_START;
            }
            break;

        /*=========== 等待启动指令 ===========*/
        case WAIT_START:
            if(Get_USART3_Data())
            {
                if(strncmp("START", (char*)USART3_RxBuffer, 5) == 0)
                {
                    /* 收到 START → 开始定点运动 */
                    Motor->Pulse_Count = 0;
                    g_RunState = GO_MID;
                }
                flashUART3();
            }
            break;

        /*=========== 【定点运动】部分 ===========*/

        /* ----- ① 从起点(0)前往中点(3200脉冲) ----- */
        case GO_MID:
            Motor->Motor_Dir = FORWARD;
            Motor_Start(Motor);
            g_RunState = WAIT_HIT;
            break;

        /* ----- ② 监控是否到达中点(2828) ----- */
        case WAIT_HIT:
            if(Motor->Pulse_Count >= 2828)
            {
                Motor_Stop();
                LED_Show_Yellow();           /* 到中点暂停 → 黄灯(采集数据) */
                g_RunState = MID_STOP;
                Stop_ms = Timer_ms;
            }
            break;

        /* ----- ③ 在中点停2秒 → 继续前往终点(6400) ----- */
        case MID_STOP:
            if((Timer_ms - Stop_ms) > 2000)
            {
                RS485_U1_Data_Print();
                LED_Show_Green();            /* 继续前进 → 绿灯 */
                Motor->Motor_Dir = FORWARD;
                Motor_Start(Motor);
                g_RunState = GO_END;
            }
            break;

        /* ----- ④ 监控是否到达终点(5650) ----- */
        case GO_END:
            if(Motor->Pulse_Count >= 5650 || hit_left)
            {
                Motor_Stop();
                LED_Show_Yellow();           /* 到终点暂停 → 黄灯(采集数据) */
                g_RunState = MID_STOP2;
                Stop_ms = Timer_ms;
                printf("pulse_count = %d\n", Motor->Pulse_Count);
            }
            break;

        /* ----- ⑤ 在终点停2秒 → 返回中点 ----- */
        case MID_STOP2:
            if((Timer_ms - Stop_ms) > 2000)
            {
                RS485_U1_Data_Print();
                LED_Show_Green();            /* 返回中点 → 绿灯 */
                Motor->Motor_Dir = BACKWARD;
                Motor_Start(Motor);
                g_RunState = BACK_MID;
            }
            break;

        /* ----- ⑥ 监控是否到达中点(2828) ----- */
        case BACK_MID:
            if(Motor->Pulse_Count <= 2828)
            {
                Motor_Stop();
                LED_Show_Yellow();           /* 回到中点暂停 → 黄灯(采集数据) */
                g_RunState = MID_STOP3;
                Stop_ms = Timer_ms;
            }
            break;

        /* ----- ⑦ 在中点停2秒 → 返回起点 ----- */
        case MID_STOP3:
            if((Timer_ms - Stop_ms) > 2000)
            {
                RS485_U1_Data_Print();
                LED_Show_Green();            /* 返回起点 → 绿灯 */
                Motor->Motor_Dir = BACKWARD;
                Motor_Start(Motor);
                g_RunState = GO_BACK;
            }
            break;

        /* ----- ⑧ 监控是否回到起点(0) ----- */
        case GO_BACK:
            if(Motor->Pulse_Count == 0 || hit_right)
            {
                Motor_Stop();
                Motor->Pulse_Count = 0;
                LED_Show_Yellow();           /* 回到起点暂停 → 黄灯(采集数据) */
                g_RunState = BACK_STOP;
                Stop_ms = Timer_ms;
            }
            break;

        /* ----- ⑨ 回到起点停2秒 → 循环再去中点 ----- */
        case BACK_STOP:
            if((Timer_ms - Stop_ms) > 2000)
            {
                RS485_U1_Data_Print();
                LED_Show_Green();
                g_RunState = GO_MID;              /* 循环：再去中点 */
            }
            break;
    }
}


/*
*************************************************************************************
*   函 数 名: TIM5_IRQHandler
*   功能说明: TIM5 PWM更新中断 — 每发一个脉冲，计一步
*   形    参：无
*   返 回 值: 无
*   描    述：前进→Pulse_Count++; 后退且>0→Pulse_Count--;
*************************************************************************************
*/
void TIM5_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM5, TIM_IT_Update) == SET)
    {
        if(Motor.Motor_Dir == FORWARD)
            Motor.Pulse_Count++;
        else if(Motor.Pulse_Count > 0)
            Motor.Pulse_Count--;

        TIM_ClearITPendingBit(TIM5, TIM_FLAG_Update);
    }
}
