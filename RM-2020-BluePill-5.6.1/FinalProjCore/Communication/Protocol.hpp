#include <FreeRTOS.h>
#include <usart.h>

namespace Protocol
{
constexpr uint8_t START_BYTE = 0xAA;

struct __attribute__((packed)) Header
{
    uint8_t begin;
    uint8_t cmd;
    uint8_t len;
};

struct __attribute__((packed)) BoardInfo
{
    uint8_t emotion;
};

/**
 * Commands from the PC
 */
enum Cmd
{
    SAY_HI = 0,
    SAY_BYE,
    SHOUT,
    LOUGH,
    CRY
};

/**
 * Information of the board (sent to the PC)
 */
enum Info
{
    HAPPY = 0,
    SAD,
    ANGRY,
    STRESSED,
    FRUSTRATED
};

/**
 * Send the current status to the PC.
 */
void sendInfo(Info &info);

void sendCustomInfo(uint8_t msg[], uint16_t len);

/**
 * User access point to the command received.
 */
const Cmd &getCmd();

/**
 * Initialize receiving task, which tries to receive data from the PC
 * continuously.
 */
void initReceiveTask();  // pseudo code: while(1){ uartReceive(); }

}  // namespace Protocol
