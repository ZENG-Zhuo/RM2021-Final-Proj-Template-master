#include <FreeRTOS.h>
#include <usart.h>

#include <RemoteControl.hpp>
namespace RemoteControl
{
static volatile int monitor;
static HAL_StatusTypeDef sta1;
void receiveUartCustomInf(uint8_t data[8])
{
    sta1 = HAL_UART_Receive_DMA(&huart1, data, 15);
}
void sendUartCustomInf(uint8_t data[8])
{
    HAL_UART_Transmit(&huart1, data, 8, 10);
}
// void deCode(uint8_t data[], RemoteControl::controlMsg msg)
// {
//     data1[0] = (data[0] | data[1] << 8) & 0x07FF;
//     data1[0] -= 1024;
//     data1[1] = (data[1] >> 3 | data[2] << 5) & 0x07FF;
//     data1[1] -= 1024;
//     data1[2] = (data[2] >> 6 | data[3] << 2 | data[4] << 10) & 0x07FF;
//     data1[2] -= 1024;
//     data1[3] = (data[4] >> 1 | data[5] << 7) & 0x07FF;
//     data1[3] -= 1024;
//     data1[4] = ((data[5] >> 4) & 0x000C) >> 2;
//     data1[5] = (data[5] >> 4) & 0x0003;
// }
int abs(int a)
{
    if (a > 0)
        return a;
    else
        return -a;
}
int last1 = 1024;
void getRemoteMsg(int16_t data1[])
{
    uint8_t tempdata[8];
    receiveUartCustomInf(tempdata);
    if ((abs(((tempdata[0] | tempdata[1] << 8) & 0x07FF) - last1) < 300))
    {
        data1[0] = (tempdata[0] | tempdata[1] << 8) & 0x07FF;
        data1[0] -= 1024;
        data1[0] = -data1[0];
        data1[1] = (tempdata[1] >> 3 | tempdata[2] << 5) & 0x07FF;
        data1[1] -= 1024;
        data1[2] =
            (tempdata[2] >> 6 | tempdata[3] << 2 | tempdata[4] << 10) & 0x07FF;
        data1[2] -= 1024;
        data1[3] = (tempdata[4] >> 1 | tempdata[5] << 7) & 0x07FF;
        data1[3] -= 1024;
        data1[4] = ((tempdata[5] >> 4) & 0x000C) >> 2;
        data1[5] = (tempdata[5] >> 4) & 0x0003;
        last1 = (tempdata[0] | tempdata[1] << 8) & 0x07FF;
    }
    // deCode(tempdata, msg);
    // monitor = msg.ch1;
}
}  // namespace RemoteControl