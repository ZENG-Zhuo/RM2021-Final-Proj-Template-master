#include <FreeRTOS.h>
#include <can.h>
#include <task.h>

#include <Chassis.hpp>
#include <RemoteControl.hpp>

#include "gpio.h"
namespace Chassis
{
const int TotalAngle = -83700;
int16_t tempSpeed[4];
int16_t speed;
int16_t zeroSpeed[4] = {0, 0, 0, 0};
uint8_t speedLevel = 0;  // 0:low 1:mid 2:high
int16_t speedLevelData[3] = {8, 8, 11};
static int16_t motorRPM[5] = {0, 0, 0, 0, 0};
int16_t motorAimRPM[5] = {0, 0, 0, 0, 0};
static volatile HAL_StatusTypeDef status;
static volatile uint32_t errCode;
CAN_RxHeaderTypeDef rxheader;
uint8_t RxData[8];
static volatile int32_t ch1, ch2, ch3;
int motor5Angle = -1;
int aimAngle;
int round = 0;
int lastAngle;
static volatile int MonitorRPM, current;
static volatile int cnt2;
static volatile int testMonitor;
bool disableMotor = false;
void canCallBack(CAN_HandleTypeDef *hcan1)
{
    HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxheader, RxData);
    motorRPM[rxheader.StdId - 0x200 - 1] = RxData[2] << 8 | RxData[3];
    MonitorRPM = motorRPM[2];
    // cnt2++;
    if (rxheader.StdId == 0x205)
        if (motor5Angle == -1)
        {
            motor5Angle = RxData[0] << 8 | RxData[1];
            lastAngle = motor5Angle;
            aimAngle = motor5Angle;
        }
        else
        {
            motor5Angle = RxData[0] << 8 | RxData[1];
            if ((motor5Angle - lastAngle) > 6000)
                round--;
            else if ((motor5Angle - lastAngle) < -6000)
                round++;
            lastAngle = motor5Angle;
        }
}

void uartCallBack(UART_HandleTypeDef *uart)
{
    // cnt2++;
    // RemoteControl::getRemoteMsg(msg);
    // ch1 = msg[0];
    // speed = speedLevelData[speedLevel];
    // tempSpeed[0] = -speed * msg[0] + speed * msg[1] + speed * msg[2];
    // tempSpeed[1] = -(+speed * msg[0] + speed * msg[1] - speed * msg[2]);
    // tempSpeed[2] = -(-speed * msg[0] + speed * msg[1] - speed * msg[2]);
    // tempSpeed[3] = +speed * msg[0] + speed * msg[1] + speed * msg[2];
    // ch1 = msg[3];
    // if (ch1 == 1)
    //     setAimSpeed(tempSpeed);
    // else if (ch1 == 2)
    // {
    //     aimAngle += msg[1] * 30;
    //     setAimSpeed(zeroSpeed);
    // }
}
void init()
{
    const uint16_t stdNum = 5;
    uint32_t stdIdArray[stdNum] = {0x201, 0x202, 0x203, 0x204, 0x205};
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterFIFOAssignment = CAN_FilterFIFO0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterMaskIdHigh = 0xFFFF;
    sFilterConfig.FilterMaskIdLow = 0xFFFF;
    sFilterConfig.FilterActivation = ENABLE;
    for (int i = 0; i < stdNum; i++)
    {
        sFilterConfig.FilterBank = i;
        sFilterConfig.FilterIdHigh =
            (((uint32_t)stdIdArray[i] << 21) & 0xffff0000) >> 16;
        sFilterConfig.FilterIdLow =
            (((uint32_t)stdIdArray[i] << 21) | CAN_ID_STD | CAN_RTR_DATA) &
            0xffff;
        HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
    }
    HAL_CAN_RegisterCallback(
        &hcan, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, canCallBack);
    // HAL_UART_RegisterCallback(&uart1,);
    HAL_CAN_ActivateNotification(&hcan, CAN_IER_FMPIE0);
    HAL_CAN_Start(&hcan);
    // status = HAL_UART_RegisterCallback(
    //     &huart1, HAL_UART_RX_COMPLETE_CB_ID, uartCallBack);
}
class PID
{
   public:
    float p, i, d, pOutput, iOutput, dOutput, totalerr, output;
    float lastErr[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    PID(float p, float i, float d, float lastValue)
    {
        this->p = p;
        this->i = i;
        this->d = d;
        // this->lastValue[0] = lastValue;
        // this->lasterr = 0.0f;
        this->totalerr = 0.0f;
    }
    PID() {}
    float update(float setPoint, float actualValue)
    {
        float error = actualValue - setPoint;
        totalerr += error;
        pOutput = -p * error;
        iOutput = -i * totalerr;
        dOutput = -d * (error - lastErr[0]);
        output = pOutput + iOutput + dOutput;
        lastErr[0] = error;
        return output;
    }
};
static int monitor222;
void sendCanCustomInf(uint32_t std_id, uint8_t TxData[], int DLC)
{
    uint32_t TxMailBox = 1;
    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.StdId = std_id;
    TxHeader.TransmitGlobalTime = DISABLE;
    TxHeader.DLC = DLC;
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailBox);
}
static volatile int stdId;
void receiveCanCustomInf(uint32_t &std_id, uint8_t RxData[8])
{
    CAN_RxHeaderTypeDef RxHeader;
    HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData);
    stdId = RxHeader.StdId;
    std_id = RxHeader.StdId;
}
void setValue(int16_t value[5])
{
    uint8_t tempdata[8];
    uint8_t tempdata1[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    monitor222 = value[2];
    for (int i = 0; i < 4; i++)
    {
        tempdata[2 * i] = value[i] >> 8;
        tempdata[2 * i + 1] = value[i] & 255;
    }
    sendCanCustomInf(0x200, tempdata, 8);
    if (!disableMotor)
    {
        tempdata1[0] = value[4] >> 8;
        tempdata1[1] = value[4] & 255;
    }
    sendCanCustomInf(0x1FF, tempdata1, 8);
}
void setAimSpeed(int16_t speed[4])
{
    for (int i = 0; i < 4; i++)
        motorAimRPM[i] = speed[i];
}
const int maxValue = 5000;
int16_t maxProtect(int16_t value)
{
    if (value > maxValue)
        return maxValue;
    else if (value > -maxValue)
        return value;
    else
        return -maxValue;
}

static volatile int cnt, cnt1;

void PIDControlSpeedLoop(void *param)
{
    PID p[5];
    float setP = 9.0f;
    float setI = 0.7f;
    float setD = 1.0f;
    int16_t temp[5];
    static int value1;
    for (int i = 0; i < 5; i++)
    {
        p[i].p = setP;
        p[i].i = setI;
        p[i].d = setD;
    }
    //p[2].p = 10.0f;
    p[4].p = 9.0f;
    while (1)
    {
        for (int i = 0; i < 5; i++)
        {
            temp[i] = maxProtect((int)p[i].update(motorAimRPM[i], motorRPM[i]));
        }
        setValue(temp);
        vTaskDelay(1);
    }
}
void PIDControlAngleLoop(void *param)
{
    PID p1;
    p1.p = 0.17f;
    p1.i = 0.0f;
    p1.d = 0.1f;
    // vTaskDelay(10);
    // aimAngle = motor5Angle;
    while (1)
    {
        motorAimRPM[4] = (int)(p1.update((float)aimAngle,
                                         (float)(motor5Angle + 8191 * round)));
    }
}

void aimSpeedLoop(void *param)
{
    int16_t msg[6];
    while (1)
    {
        cnt2++;
        RemoteControl::getRemoteMsg(msg);
        ch1 = msg[4];
        ch2 = msg[5];
        ch3 = msg[1];

        // ch1 = msg[3];
        if (ch1 == 2)
        {
            if (ch2 == 2)
                speedLevel = 0;
            else if (ch2 == 3)
                speedLevel = 1;
            else if (ch2 == 1)
                speedLevel = 2;
            speed = speedLevelData[speedLevel];
            tempSpeed[0] = -speed * msg[0] + speed * msg[1] + speed * msg[2];
            tempSpeed[1] = -(+speed * msg[0] + speed * msg[1] - speed * msg[2]);
            tempSpeed[2] = -(-speed * msg[0] + speed * msg[1] - speed * msg[2]);
            tempSpeed[3] = +speed * msg[0] + speed * msg[1] + speed * msg[2];
            setAimSpeed(tempSpeed);
        }
        else if (ch1 == 3)
        {
            if (ch2 == 2)
            {
                aimAngle -= msg[1] * 3;
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
                setAimSpeed(zeroSpeed);
            }
            else if (ch2 == 3)
            {
                disableMotor = true;
            }
            else if (ch2 == 1)
            {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_SET);
            }
        }
        else if (ch1 == 1)
        {
            if (ch2 == 2)
            {
                aimAngle -= msg[1] * 3;
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
                setAimSpeed(zeroSpeed);
            }
            else if (ch2 == 3)
            {
                int tempangle;
                aimAngle = motor5Angle;
                tempangle = motor5Angle;
                round = 0;
                disableMotor = false;
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
                for (int i = 0; i > (-2000); i -= 10)
                {
                    aimAngle = tempangle + i;
                    vTaskDelay(1);
                }
            }
            else if (ch2 == 1)
            {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_RESET);
            }
            aimAngle -= msg[1] * 3;
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
        }
        // motorRPM[0] = 4 * (monitor - 1024);
        vTaskDelay(10);
    }
}

void chassisStart(void *param) { HAL_CAN_Start(&hcan); }
}  // namespace Chassis