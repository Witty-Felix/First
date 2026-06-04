#include "Serial.h"

uint8_t USART3_RxBuffer[USART3_RX_BUFFER_SIZE];
volatile uint16_t USART3_RxCount = 0;
volatile uint16_t USART3_RxFinished = 0;

/* 全局变量 */
volatile uint8_t Serial_RxFlag;
volatile uint8_t Serial_RxData;

/*
*************************************************************************************
*   函 数 名: DMA_USART3_RX_Config
*   功能说明: USART3-DMA接收初始化（DMA1通道3）
*   形    参：无
*   返 回 值: 无
*   描    述：配置DMA1_CH3搬运USART3->DR到USART3_RxBuffer
*           ：Normal模式，单字节传输，目的地址自增
*           ：不开启DMA中断（配合定时查询或空闲中断使用）
*************************************************************************************
*/
static void DMA_USART3_RX_Config(void)
{
    /*开启时钟*/
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_InitTypeDef dma;
    DMA_DeInit(DMA1_Channel3);  //恢复出厂设置

    //2、从哪搬到哪
    dma.DMA_PeripheralBaseAddr  = (uint32_t)&(USART3->DR);    // 源地址
    dma.DMA_MemoryBaseAddr      = (uint32_t)USART3_RxBuffer;  // 目的地址
    dma.DMA_DIR                 = DMA_DIR_PeripheralSRC;      // 方向：外设->内存
    //3、一次搬多少
    dma.DMA_BufferSize          = 256;
    dma.DMA_PeripheralInc       = DMA_PeripheralInc_Disable;  // 源地址不自增
    dma.DMA_MemoryInc           = DMA_MemoryInc_Enable;       // 目的地址自增
    dma.DMA_PeripheralDataSize  = DMA_PeripheralDataSize_Byte;// 每次搬一个字节
    dma.DMA_MemoryDataSize      = DMA_MemoryDataSize_Byte;
    //3、其他设置
    dma.DMA_Mode                = DMA_Mode_Normal;            // 正常模式（不是循环）
    dma.DMA_Priority            = DMA_Priority_Medium;        // 中等优先级
    dma.DMA_M2M                 = DMA_M2M_Disable;            // 不是内存到内存

    DMA_Init(DMA1_Channel3, &dma);

    DMA_Cmd(DMA1_Channel3, ENABLE);                           // 启动搬运工
}

/* 
*************************************************************************************
*   函 数 名: USART3_Enable_DMA_RX
*   功能说明: 使能USART3的DMA接收请求
*   形    参：无
*   返 回 值: 无
*   描    述：调用后USART3收到数据时自动触发DMA搬运
*           ：需先调用DMA_USART3_RX_Config()配置DMA通道
*************************************************************************************
*/
static void USART3_Enable_DMA_RX(void)
{
    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
}

/*
*************************************************************************************
*   函 数 名: Serial_Init
*   功能说明: 串口初始化
*   引    脚: TX<PC10> | RX<PC11> (USART3部分重映射)
*   形    参：无
*   返 回 值: 无
*   描    述：开启USART3、GPIO、AFIO时钟
*           ：配置TX为复用推挽、RX为上拉输入
*           ：设置波特率115200、8N1
*           ：使能接收中断(NVIC抢占1/子优先级1)
*************************************************************************************
*/
void Serial_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Mode   = GPIO_Mode_AF_PP;
    gpio.GPIO_Pin    = GPIO_Pin_10;
    gpio.GPIO_Speed  = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpio);

    gpio.GPIO_Mode   = GPIO_Mode_IPU;
    gpio.GPIO_Pin    = GPIO_Pin_11;
    gpio.GPIO_Speed  = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpio);

    USART_InitTypeDef usart;
    usart.USART_BaudRate            = 115200;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_WordLength          = USART_WordLength_8b;
    USART_Init(USART3, &usart);

    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel                   = USART3_IRQn;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 1;
    NVIC_Init(&nvic);

    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);  // 开启空闲中断（一帧数据收完时触发）

    USART_Cmd(USART3, ENABLE);
    USART_ClearFlag(USART3, USART_FLAG_TC);  // 清除发送完成标志，避免一上来就误触发

    DMA_USART3_RX_Config();
    USART3_Enable_DMA_RX();
}

/*
*************************************************************************************
*   函 数 名: Serial_SendByte
*   功能说明: 串口发送一个字节
*   形    参：Data - 待发送数据
*   返 回 值: 无
*************************************************************************************
*/
void Serial_SendByte(uint8_t Data)
{
    USART_SendData(USART3, Data);
    while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
}

/*
*************************************************************************************
*   函 数 名: Serial_SendArray
*   功能说明: 串口发送字节数组
*   形    参：Array - 数据缓冲区指针 | Length - 发送长度
*   返 回 值: 无
*************************************************************************************
*/
void Serial_SendArray(uint8_t* Array, uint16_t Length)
{
    uint16_t i;
    for(i = 0; i < Length; i++)
        Serial_SendByte(Array[i]);
}

/*
*************************************************************************************
*   函 数 名: Serial_SendString
*   功能说明: 串口发送字符串
*   形    参：str - 字符串指针
*   返 回 值: 无
*************************************************************************************
*/
void Serial_SendString(char* str)
{
    while(*str != '\0')
        Serial_SendByte(*str++);
}

/*
*************************************************************************************
*   函 数 名: MyPow
*   功能说明: 整数次幂运算（内部使用）
*   形    参：Base - 底数 | Count - 指数
*   返 回 值: 幂运算结果
*************************************************************************************
*/
static uint32_t MyPow(uint32_t Base, uint32_t Count)
{
    uint32_t ret = 1;
    while(Count--) ret *= Base;
    return ret;
}

/*
*************************************************************************************
*   函 数 名: Serial_SendNum
*   功能说明: 串口发送十进制数字
*   形    参：Num - 待发送数字 | Length - 显示位数
*   返 回 值: 无
*************************************************************************************
*/
void Serial_SendNum(uint32_t Num, uint16_t Length)
{
    uint16_t i;
    for(i = 0; i < Length; i++)
        Serial_SendByte(Num / MyPow(10, Length - i - 1) % 10 + '0');
}

/*
*************************************************************************************
*   函 数 名: fputc
*   功能说明: printf重定向
*   形    参：ch - 待打印字符 | f - 文件指针
*   返 回 值: 发送的字符
*************************************************************************************
*/
int fputc(int ch, FILE* f)
{
    Serial_SendByte(ch);
    return ch;
}

/* 
*************************************************************************************
*   函 数 名: Get_USART3_Data
*   功能说明: 获取USART3 DMA接收数据（轮询调用）
*   形    参：无
*   返 回 值: 1 - 收到新数据并已回显；0 - 无新数据
*   描    述：当USART3_RxFinished标志置位时，打印接收数据，
*           ：重置DMA计数器并使能DMA，准备接收下一帧
*************************************************************************************
*/
uint8_t Get_USART3_Data(void)
{
    if(USART3_RxFinished == 1)
    {
        USART3_RxFinished = 0;

        // 把收到的数据打印出来（回显到串口）
        printf("usart3_rcv = %s\n", USART3_RxBuffer);
        // 重新设置DMA计数为256，准备接收下一帧
        DMA_SetCurrDataCounter(DMA1_Channel3, USART3_RX_BUFFER_SIZE);
        // 重新启动DMA
        DMA_Cmd(DMA1_Channel3, ENABLE);

        return 1;   //告诉你，有数据
    }
    return 0;       //没数据
}

/*
*************************************************************************************
*   函 数 名: flashUART3
*   功能说明: 清空USART3接收缓冲区
*   形    参：无
*   返 回 值: 无
*   描    述：将USART3_RxBuffer全部置零，用于接收前/出错时复位
*************************************************************************************
*/
void flashUART3(void)
{
    memset(USART3_RxBuffer, 0, USART3_RX_BUFFER_SIZE);
}


/*
*************************************************************************************
*   函 数 名: USART3_DMA_Rx_DataReady
*   功能说明: USART3-DMA接收完成处理
*   形    参：无
*   返 回 值: 无
*   描    述：暂停DMA → 计算实际接收字节数 → 置位接收完成标志
*************************************************************************************
*/
static void USART3_DMA_Rx_DataReady(void)
{
    // 暂停DMA，停止搬运（不然DMA还在跑，不好算长度）             
    DMA_Cmd(DMA1_Channel3, DISABLE);                              
    // 计算收到了多少个字节                                       
    // DMA总共256字节 - DMA还剩的字节数 = 实际收到的字节数        
    USART3_RxCount =  USART3_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA1_Channel3);                                                    
                                                                  
    // 立标志：告诉主循环"有数据来了，快来取"                     
    USART3_RxFinished = 1;  
}

/*
*************************************************************************************
*   函 数 名: USART3_IRQHandler
*   功能说明: 串口中断服务函数
*   形    参：无
*   返 回 值: 无
*************************************************************************************
*/
void USART3_IRQHandler(void)
{
    if(USART_GetITStatus(USART3, USART_IT_IDLE) == SET)
    {
        // 这就是STM32清除空闲中断标志的固定写法，照抄就完事了。
        volatile uint16_t temp = USART3->SR;
        temp = USART3->DR;
        (void)temp;

        USART3_DMA_Rx_DataReady();
    }
}
