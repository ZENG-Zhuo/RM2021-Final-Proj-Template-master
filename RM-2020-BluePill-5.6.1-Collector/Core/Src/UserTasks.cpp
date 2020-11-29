/**
 * @file UserTasks.cpp
 * @author Will (phliuab@connect.ust.hk)
 * @brief File storing all user tasks
 * @version 0.1
 * @date 2020-09-08
 *
 * @copyright Copyright (c) 2020
 *
 */

#include <can.h>

#include <Chassis.hpp>
#include <Protocol.hpp>

#include "FreeRTOS.h"
#include "gpio.h"
#include "main.h"
#include "string.h"
#include "task.h"
#include "usart.h"
StaticTask_t xBlinkTaskTCB;
StackType_t uxBlinkTaskStack[64];
void blinky(void *param)
{
    while (true)
    {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        vTaskDelay(50);
    }
}

StaticTask_t uartTaskTCB;
StackType_t uartTaskStack[128];
static volatile uint8_t msg[] = "IKEMURA";
static volatile uint8_t msg2[] = "KEI";
void CVCommunicationTask(void *param)
{
    HAL_UART_Init(&huart1);
    uint16_t strLen = strlen((char *)msg);
    uint16_t strLen2 = strlen((char *)msg2);

    while (true)
    {
        Protocol::sendCustomInfo((uint8_t *)msg2, strLen2);
        vTaskDelay(800);
        Protocol::sendCustomInfo((uint8_t *)msg, strLen);
    }
}

StaticTask_t canTaskTCB;
StackType_t canTaskStack[512];
StaticTask_t uartTaskTCB1;
StackType_t uartTaskStack1[512];
StaticTask_t uartTaskTCB2;
StackType_t uartTaskStack2[512];
StaticTask_t uartTaskTCB3;
StackType_t uartTaskStack3[512];
uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};

void canTask(void *param)
{
    CAN_TxHeaderTypeDef header = {0, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE};
    HAL_CAN_Start(&hcan);
    uint32_t mailbox;

    while (1)
    {
        HAL_CAN_AddTxMessage(&hcan, &header, data, &mailbox);
        vTaskDelay(1);
    }
}
void testValue(void *param)
{
    while (1)
    {
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);
        vTaskDelay(2000);
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);
        vTaskDelay(2000);
    }
}

extern "C"
{
    void startUserTasks()
    {
        // xTaskCreateStatic(
        //     blinky, "blink", 64, NULL, 0, uxBlinkTaskStack, &xBlinkTaskTCB);
        
        // xTaskCreateStatic(CVCommunicationTask,
        //                   "serial",
        //                   128,
        //                   NULL,
        //                   0,
        //                   uartTaskStack,
        //                   &uartTaskTCB);
        Chassis::init();
        // xTaskCreateStatic(
        //     canTask, "can", 128, NULL, 0, canTaskStack, &canTaskTCB);
        // xTaskCreateStatic(
        //     testValue, "test1", 512, NULL, 1 , uartTaskStack2,
        //     &uartTaskTCB2);
        xTaskCreateStatic(Chassis::PIDControlSpeedLoop,
                          "test",
                          512,
                          NULL,
                          2,
                          uartTaskStack1,
                          &uartTaskTCB1);
        xTaskCreateStatic(Chassis::halfTest,
                          "test2",
                          512,
                          NULL,
                          2,
                          uartTaskStack2,
                          &uartTaskTCB2);
        xTaskCreateStatic(Chassis::PIDControlAngleLoop,
                          "test3",
                          512,
                          NULL,
                          2,
                          uartTaskStack3,
                          &uartTaskTCB3);
    }
}
