#include "LED.h"
#include "LimSwc.h"
#include "Serial.h"
#include "Timer.h"
#include "Motor.h"
#include "RS485.h"
#include "System.h"
#include "Delay.h"

uint16_t count;

int main(void)
{
    /* 系统初始化 */
    System_Init();

    /* 外设初始化 */
    LED_Init();
    LimSwc_Init();
    Serial_Init();
    Timer_Init();
    Motor_Init(&Motor);
    RS485_U1_Init(9600);
    RS485_U2_Init(9600);

    /* 应用启动 */
    g_RunState = HOME_INIT;
    // Motor_Start(&Motor);
    printf("Motor Starts!\n");
    LED_Show_Green();                    /* 初始状态：无障碍物 → 绿灯 */

    if(1)
    {
        while (1)
        {
            /* 10ms 一次采集数据 */
            if(Timer_Poll_Flag == 10)
            {
                Timer_Poll_Flag %= 10;  // 为10清零
                count++;
                RS485_U1_Poll();
            }

            if(count == 100)
            {
                count %= 100;
                printf("DIST = %d\n", RADAR_RX_DIST);
            }

            Motor_Check_State(&Motor);
            
        }
    }
    else
    {
        Motor_EN(DIS);
    }
}
