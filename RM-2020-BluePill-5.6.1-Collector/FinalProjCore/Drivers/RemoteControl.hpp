#include <FreeRTOS.h>
#include <usart.h>
namespace RemoteControl
{
void init();
struct controlMsg
{
    int ch1, ch2, ch3, ch4, sw1, sw2;
};
uint16_t getChannelValue(uint8_t channelID);
void getRemoteMsg(int16_t data1[]);
void sendUartCustomInf(uint8_t data[8]);
void receiveUartCustomInf(uint8_t data[8]);
}  // namespace RemoteControl
