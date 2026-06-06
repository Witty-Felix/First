#include "RS485.h"
#include "Delay.h"

/* 全局变量 */
volatile RS485_TypeDef g_RS485 = {0};
volatile uint16_t      RADAR_RX_DIST;
volatile uint16_t      RADAR_RX_TEMP;
/* 滑动平均参数 */
#define MA_WINDOW_SIZE       3

static uint16_t s_maDist[MA_WINDOW_SIZE];   /* 距离滑动缓冲区 */
static uint16_t s_maTemp[MA_WINDOW_SIZE];   /* 温度滑动缓冲区 */

//灯带的数组：
static uint8_t rgb_send_cmd[21] = {
        0xDD,0x55,0xEE,     //帧头
        0x00,0x01,          //设备地址
        0x00,0x01,0x00,0x99,0x01,0x00,0x00,0x00,0x03,0x01,0x2C,
        0x00,0x00,0x00,     //控制颜色
        0xAA,0xBB           //帧尾（固定）
};

//灯带实现的各种颜色
static const uint8_t rgb_no_light[3]     = 	{0x00,0x00,0x00};
static const uint8_t rgb_red_light[3]    =  {0xF0,0x00,0x00};
static const uint8_t rgb_blue_light[3]   = 	{0x00,0x00,0xFF};
static const uint8_t rgb_green_light[3]  = 	{0x00,0xFF,0x00};
static const uint8_t rgb_yellow_light[3] = 	{0xFF,0xFF,0x00};
static const uint8_t rgb_orange_light[3] = 	{0xFF,0x45,0x00};
static const uint8_t rgb_color_light[3]  =  {0xFF,0xFF,0xFF};   //自定义颜色


/* CRC校验表 */
static const uint8_t s_CRCHi[256] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};
static const uint8_t s_CRCLo[256] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04,
    0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
    0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10,
    0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C,
    0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,
    0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
    0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
    0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
    0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

/*
*************************************************************************************
*   函 数 名: CRC16_Modbus
*   功能说明: Modbus CRC16校验
*   形    参：data - 数据指针 | len - 长度
*   返 回 值: 16位CRC校验值
*************************************************************************************
*/
static uint16_t CRC16_Modbus(uint8_t* data, uint16_t len)
{
    uint8_t crc_hi = 0xFF;
    uint8_t crc_lo = 0xFF;
    uint16_t index;
    while(len--)
    {
        index = crc_hi ^ *data++;
        crc_hi = crc_lo ^ s_CRCHi[index];
        crc_lo = s_CRCLo[index];
    }
    return ((uint16_t)crc_hi << 8 | crc_lo);
}

/*
*************************************************************************************
*   函 数 名: MovingAvg_Filter
*   功能说明: 滑动平均滤波 — 取最近N个值的平均值
*   形    参：buf - 缓冲区指针 | new_val - 新值
*   返 回 值: 滤波后的值
*   描    述：先进先出移位，求窗口内平均值
*************************************************************************************
*/
static uint16_t MovingAvg_Filter(uint16_t* buf, uint16_t new_val)
{
    uint32_t sum = 0;
    uint8_t i;

    /* 左移：丢掉最老的，空出最后一个位置 */
    for(i = 0; i < MA_WINDOW_SIZE - 1; i++)
        buf[i] = buf[i + 1];
    buf[MA_WINDOW_SIZE - 1] = new_val;

    /* 求和 */
    for(i = 0; i < MA_WINDOW_SIZE; i++)
        sum += buf[i];

    return (uint16_t)(sum / MA_WINDOW_SIZE);
}


/*
*************************************************************************************
*   函 数 名: RS485_U1_Init
*   功能说明: RS485初始化(USART1)
*   引    脚: TX<PA9> | RX<PA10> | DE<PA12>
*   形    参：bound - 波特率
*   返 回 值: 无
*   描    述：开启USART1及GPIOA时钟
*           ：配置TX复用推挽、RX上拉输入、DE推挽输出
*           ：设置8N1模式，使能接收中断(NVIC抢占1/子优先级1)
*************************************************************************************
*/
void RS485_U1_Init(uint32_t bound)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin    = GPIO_Pin_9;
    gpio.GPIO_Mode   = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed  = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin    = GPIO_Pin_10;
    gpio.GPIO_Mode   = GPIO_Mode_IPU;
    gpio.GPIO_Speed  = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin    = GPIO_Pin_12;
    gpio.GPIO_Mode   = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed  = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    USART_InitTypeDef usart;
    usart.USART_BaudRate            = bound;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_WordLength          = USART_WordLength_8b;
    USART_Init(USART1, &usart);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel                   = USART1_IRQn;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 1;
    NVIC_Init(&nvic);

    USART_Cmd(USART1, ENABLE);
    GPIO_ResetBits(GPIOA, GPIO_Pin_12);    /* 默认接收模式 */
}

/*
*************************************************************************************
*   函 数 名: RS485_U1_SendArray
*   功能说明: RS485--USART1 发送字节数组
*   形    参：array - 数据缓冲区指针 | length - 发送长度
*   返 回 值: 无
*   描    述：DE置高切为发送模式，发送完延时后切回接收模式
*************************************************************************************
*/
void RS485_U1_SendArray(uint8_t* array, uint16_t length)
{
    uint16_t i;
    GPIO_SetBits(GPIOA, GPIO_Pin_12);      /* 切换到发送 */
    Delay_ms(3);
    for(i = 0; i < length; i++)
    {
        USART_SendData(USART1, array[i]);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    }
    while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
    Delay_ms(3);
    GPIO_ResetBits(GPIOA, GPIO_Pin_12);    /* 切换回接收 */
}

/*
*************************************************************************************
*   函 数 名: RS485_U2_Init
*   功能说明: RS485初始化(USART2)
*   引    脚: TX<PD5> | RX<PD6> | DE<PD7>
*   形    参：bound - 波特率
*   返 回 值: 无
*   描    述：开启USART2及GPIOA、AFIO时钟
*           ：配置TX复用推挽、RX上拉输入、DE推挽输出，设置8N1模式
*************************************************************************************
*/
void RS485_U2_Init(uint32_t bound)
{
    /*开启时钟*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    /*引脚配置*/
    GPIO_InitTypeDef gpio;
    
    //PD7 -> 使能引脚
    gpio.GPIO_Pin    = GPIO_Pin_7;  
    gpio.GPIO_Mode   = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed  = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &gpio);

    //PD5 -> Tx
    gpio.GPIO_Pin    = GPIO_Pin_5;
    gpio.GPIO_Mode   = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed  = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &gpio);

    //PD6 -> Rx
    gpio.GPIO_Pin    = GPIO_Pin_6;
    gpio.GPIO_Mode   = GPIO_Mode_IPU;
    gpio.GPIO_Speed  = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &gpio);

    /*引脚重映射*/
    GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);

    /*串口初始化*/
    USART_InitTypeDef usart;
    usart.USART_BaudRate            = bound;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART2, &usart);

    /*开启接收中断并使能串口*/
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART2, ENABLE);

    USART_ClearFlag(USART2, USART_FLAG_TC);

    /*默认接收状态*/
    GPIO_ResetBits(GPIOD, GPIO_Pin_7);
}

/*
*************************************************************************************
*   函 数 名: RS485_U2_SendArray
*   功能说明: RS485--USART2 发送字节数组
*   形    参：array - 数据缓冲区指针 | length - 发送长度
*   返 回 值: 无
*   描    述：DE置高切为发送模式，发送完延时后切回接收模式
*************************************************************************************
*/
void RS485_U2_SendArray(uint8_t* array, uint16_t length)
{
    uint8_t i;
    GPIO_SetBits(GPIOD, GPIO_Pin_7);    /* 切换到发送 */
    Delay_ms(3);
    for(i = 0; i < length; i++)
    {
        USART_SendData(USART2, array[i]);
        while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    }
    while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
    Delay_ms(3);
    GPIO_ResetBits(GPIOD, GPIO_Pin_7);  /* 切换回接收 */
}

/*
*************************************************************************************
*   函 数 名: U1_RX_IRQn
*   功能说明: USART1接收中断处理（内部函数）
*   形    参：ch - 接收到的字节
*   返 回 值: 无
*   描    述：地址过滤 > Modbus CRC校验 > 提取雷达距离/温度
*************************************************************************************
*/
static void U1_RX_IRQn(uint8_t ch)
{
    /* 首字节：地址过滤 */
    if(g_RS485.U1_RxCnt == 0)
    {
        if(RADAR_ADDR == ch)
            g_RS485.U1_RxBuf[g_RS485.U1_RxCnt++] = ch;
    }
    /* 后续字节：收满后自动CRC校验 */
    else if(g_RS485.U1_RxCnt < U1_RX_BUF_SIZE)
    {
        g_RS485.U1_RxBuf[g_RS485.U1_RxCnt++] = ch;

        if(g_RS485.U1_RxCnt >= 5)
        {
            uint8_t exp_len = g_RS485.U1_RxBuf[2] + 5;
            uint16_t calc_crc, recv_crc;

            if(exp_len > U1_RX_BUF_SIZE)      /* 防畸形报文：期望长度超出缓冲区上限 */
            {                                 /* 若不拦截，RxCnt 永远达不到 exp_len，通信死锁 */
                g_RS485.U1_Busy = 0;          /* 释放忙锁，允许下一帧重试 */
                g_RS485.U1_RxCnt = 0;         /* 清接收计数，丢弃已收数据 */
                g_RS485.U1_Err = 1;           /* 置错误标志，供 RS485_U1_Data_Print() 上报 */
                return;
            }

            if(g_RS485.U1_RxCnt >= exp_len)
            {
                calc_crc = CRC16_Modbus((uint8_t*)g_RS485.U1_RxBuf, exp_len - 2);
                recv_crc = (g_RS485.U1_RxBuf[g_RS485.U1_RxCnt - 2] << 8)
                            | g_RS485.U1_RxBuf[g_RS485.U1_RxCnt - 1];

                if(calc_crc == recv_crc)
                {
                    RADAR_RX_DIST = (g_RS485.U1_RxBuf[3] << 8) | g_RS485.U1_RxBuf[4];
                    RADAR_RX_TEMP = (g_RS485.U1_RxBuf[5] << 8) | g_RS485.U1_RxBuf[6];

                    /* 滑动平均 */
                    RADAR_RX_DIST = MovingAvg_Filter(s_maDist, RADAR_RX_DIST);
                    RADAR_RX_TEMP = MovingAvg_Filter(s_maTemp, RADAR_RX_TEMP);

                    g_RS485.U1_RX_FLAG = 1;
                }
                else
                    g_RS485.U1_Err = 1;

                g_RS485.U1_Busy = 0;
                g_RS485.U1_RxCnt = 0;
            }
        }
    }
}

/*
*************************************************************************************
*   函 数 名: USART1_IRQHandler
*   功能说明: USART1中断服务函数
*   形    参：无
*   返 回 值: 无
*   描    述：接收数据送入U1_RX_IRQn处理
*************************************************************************************
*/
void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
    {
        uint8_t ch = USART_ReceiveData(USART1);
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
        U1_RX_IRQn(ch);
    }
}

/*
*************************************************************************************
*   函 数 名: RS485_U1_ReadRegister
*   功能说明: 读取传感器寄存器
*   形    参：slave_addr - 从机地址 | reg_addr - 寄存器地址
*               | reg_num - 读取数量
*   返 回 值: 无
*   描    述：构建Modbus 0x03读寄存器指令，CRC校验后发送
*************************************************************************************
*/
void RS485_U1_ReadRegister(uint8_t slave_addr, uint16_t reg_addr, uint16_t reg_num)
{
    uint8_t buf[8];
    uint16_t crc;

    /*忙检测：前一次通信未完成则等待(超时300ms强制复位)*/
    if(g_RS485.U1_Busy)
    {
        if((Timer_ms - g_RS485.U1_TxTime) > 300)
        {
            g_RS485.U1_Busy  = 0;
            g_RS485.U1_RxCnt = 0;
            g_RS485.U1_Err   = 0;
            /* 清USART错误标志，防止ORE/FE/NE锁死接收 */
            USART_ClearFlag(USART1, USART_FLAG_ORE);
            USART_ClearFlag(USART1, USART_FLAG_FE);
            USART_ClearFlag(USART1, USART_FLAG_NE);
        }
        else
            return;
    }

    buf[0] = slave_addr;
    buf[1] = 0x03;
    buf[2] = (reg_addr >> 8) & 0xFF;
    buf[3] = reg_addr & 0xFF;
    buf[4] = (reg_num >> 8) & 0xFF;
    buf[5] = reg_num & 0xFF;

    crc = CRC16_Modbus(buf, 6);
    buf[6] = (crc >> 8) & 0xFF;
    buf[7] = crc & 0xFF;

    g_RS485.U1_Busy   = 1;
    g_RS485.U1_RxCnt  = 0;
    g_RS485.U1_Err    = 0;
    g_RS485.U1_TxTime = Timer_ms;
    RS485_U1_SendArray(buf, 8);
}

/*
*************************************************************************************
*   函 数 名: RS485_U1_Poll
*   功能说明: 轮询读雷达（只发指令，不打印）
*   形    参：无
*   返 回 值: 无
*   描    述：由定时器每200ms触发
*************************************************************************************
*/
void RS485_U1_Poll(void)
{
    RS485_U1_ReadRegister(RADAR_ADDR, 0x0101, 2);
}

/*
*************************************************************************************
*   函 数 名: RS485_U1_GetRxFlag
*   功能说明: 获取数据接收完成标志
*   形    参：无
*   返 回 值: 1 - 收到新数据，0 - 无数据
*************************************************************************************
*/
uint8_t RS485_U1_GetRxFlag(void)
{
    if(g_RS485.U1_RX_FLAG)
    {
        g_RS485.U1_RX_FLAG = 0;
        return 1;
    }
    return 0;
}

/*
*************************************************************************************
*   函 数 名: RS485_U1_Data_Print
*   功能说明: 读取并打印雷达距离/温度
*   形    参：无
*   返 回 值: 无
*   描    述：发送0x03读寄存器指令
*           ：收到数据后打印，超时或CRC错误打印提示
*************************************************************************************
*/
void RS485_U1_Data_Print(void)
{
    uint32_t start;

    RS485_U1_ReadRegister(RADAR_ADDR, 0x0101, 2);

    /* 等待回复（超时 100ms） */
    start = Timer_ms;
    while (!g_RS485.U1_RX_FLAG && !g_RS485.U1_Err)
    {
        if ((Timer_ms - start) > 100)
        {
            printf("雷达无应答\n");
            g_RS485.U1_Busy = 0;
            return;
        }
    }
    
    if (g_RS485.U1_RX_FLAG)
    {
        g_RS485.U1_RX_FLAG = 0;
        printf("Dist: %d.%dcm,Temp: %d.%d°C\n", RADAR_RX_DIST / 10,
            RADAR_RX_DIST % 10, RADAR_RX_TEMP / 10, RADAR_RX_TEMP % 10);
    }
    else if (g_RS485.U1_Err)
    {
        printf("数据校验失败\n");
        g_RS485.U1_Err = 0;
    }
}

/*
  *************************************************************************************
  *   灯带控制函数组
  *   功能说明: 通过RS485(UART2)发送21字节控制帧，驱动外部灯带显示指定颜色
  *   形    参：无
  *   返 回 值: 无
  *   描    述：rgb_send_cmd[]为固定帧(帧头+地址+帧尾)
  *           ：各函数仅替换偏移[16、17、18] 3处的
  *           ：RGB值(rgb_xxx_light数组)，调用RS485_U2_SendArray发送
  *   列    表：
  *           ：LED_Show_no     - 熄灭
  *           ：LED_Show_Red    - 红色
  *           ：LED_Show_Blue   - 蓝色
  *           ：LED_Show_Green  - 绿色
  *           ：LED_Show_Yellow - 黄色
  *           ：LED_Show_Color  - 自定义色
  *************************************************************************************
  */
void LED_Show_no(void)
{
    memcpy(&rgb_send_cmd[16], rgb_no_light, 3);
    RS485_U2_SendArray(rgb_send_cmd, 21);   
}
void LED_Show_Red(void)
{
    memcpy(&rgb_send_cmd[16], rgb_red_light, 3);
    RS485_U2_SendArray(rgb_send_cmd, 21);   
}
void LED_Show_Blue(void)
{
    memcpy(&rgb_send_cmd[16], rgb_blue_light, 3);
    RS485_U2_SendArray(rgb_send_cmd, 21);   
}
void LED_Show_Green(void)
{
    memcpy(&rgb_send_cmd[16], rgb_green_light, 3);
    RS485_U2_SendArray(rgb_send_cmd, 21);   
}
void LED_Show_Yellow(void)
{
    memcpy(&rgb_send_cmd[16], rgb_yellow_light, 3);
    RS485_U2_SendArray(rgb_send_cmd, 21);   
}
void LED_Show_Orange(void)
{
    memcpy(&rgb_send_cmd[16], rgb_orange_light, 3);
    RS485_U2_SendArray(rgb_send_cmd, 21);   
}
void LED_Show_Color(void)
{
    memcpy(&rgb_send_cmd[16], rgb_color_light, 3);
    RS485_U2_SendArray(rgb_send_cmd, 21);   
}
