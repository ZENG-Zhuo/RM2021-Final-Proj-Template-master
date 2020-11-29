#include <FreeRTOS.h>
#include <can.h>
#include <task.h>
namespace Chassis
{
void testMotor(void* param);
void PIDControlSpeedLoop(void* param);
void currentSpeedLoop(void* param);
void init();
void aimSpeedLoop(void *param);
void PIDControlAngleLoop(void *param);
void PIDControlAngleLoop(void *param);
void setAimSpeed(int16_t speed[4]);
void halfTest(void *param);
// void canCallBack(CAN_HandleTypeDef *hcan1);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1);
}  // namespace Chassis