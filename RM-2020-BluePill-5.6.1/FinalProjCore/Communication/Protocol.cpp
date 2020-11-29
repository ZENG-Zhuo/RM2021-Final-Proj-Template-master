#include <Protocol.hpp>

#include "task.h"

namespace Protocol
{
static Cmd commandRx;
static Header *receivedHeader;

void sendInfo(Info &info)
{
    // Send info.
}

void sendCustomInfo(uint8_t msg[], uint16_t len)
{
    HAL_UART_Transmit(&huart1, msg, len, 1);
}

const Cmd &getCmd() { return commandRx; }

StaticTask_t receiveTaskTCB;
StackType_t receiveTaskStack[128];
static void receiveTask(void *param)
{
    HAL_UART_Init(&huart1);

    uint8_t header[3];

    while (1)
    {
        HAL_UART_Receive(&huart1, header, 3, 10);

        receivedHeader = (Header *)header;

        if (receivedHeader->begin != START_BYTE)
            continue;

        commandRx = (Cmd)receivedHeader->cmd;

        vTaskDelay(1);
    }
}

void initReceiveTask()
{
    xTaskCreateStatic(
        receiveTask, "CV", 128, NULL, 10, receiveTaskStack, &receiveTaskTCB);
}

}  // namespace Protocol
