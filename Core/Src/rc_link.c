#include "rc_link.h"
#include <string.h>

/* Must match the ESP32-side FCFrame struct layout exactly:
 * [0]     0xFF  start byte 1
 * [1]     0xFE  start byte 2
 * [2..3]  int16 leftX   (roll)
 * [4..5]  int16 leftY   (pitch)
 * [6..7]  int16 rightX  (yaw)
 * [8..9]  int16 rightY  (aux)
 * [10]    uint8 throttle
 * [11]    uint8 buttons
 * [12]    uint8 checksum (XOR of bytes 2..11)
 */
#pragma pack(push, 1)
typedef struct {
    uint8_t  start1;
    uint8_t  start2;
    int16_t  leftX;
    int16_t  leftY;
    int16_t  rightX;
    int16_t  rightY;
    uint8_t  throttle;
    uint8_t  buttons;
    uint8_t  checksum;
} FCFrame_t;
#pragma pack(pop)

#define FRAME_SIZE      ((uint8_t)sizeof(FCFrame_t)) /* 13 bytes */
#define SYNC_BYTE_1     0xFF
#define SYNC_BYTE_2     0xFE

typedef enum {
    RX_WAIT_SYNC1 = 0,
    RX_WAIT_SYNC2,
    RX_COLLECT_PAYLOAD
} RxState_t;

volatile RC_Command_t rc_command = {0};

static UART_HandleTypeDef *s_huart;
static uint8_t   rx_byte;                 /* single-byte IT receive target */
static uint8_t   frame_buf[FRAME_SIZE];
static uint8_t   frame_idx;
static RxState_t rx_state = RX_WAIT_SYNC1;

/* XOR of bytes 2..11 (leftX..buttons) -- same formula as the ESP32 side */
static uint8_t ComputeChecksum(const uint8_t *frame)
{
    uint8_t chk = 0;
    for (uint8_t i = 2; i < FRAME_SIZE - 1; i++) {
        chk ^= frame[i];
    }
    return chk;
}

void RC_Init(UART_HandleTypeDef *huart)
{
    s_huart   = huart;
    rx_state  = RX_WAIT_SYNC1;
    frame_idx = 0;

    /* UART7's NVIC interrupt is now enabled by CubeMX's generated
     * HAL_UART_MspInit (regenerated after enabling "USART7 global
     * interrupt" in the .ioc NVIC settings), so no manual NVIC call
     * is needed here anymore. */
    HAL_UART_Receive_IT(s_huart, &rx_byte, 1);
}

uint8_t RC_IsLinkValid(uint32_t timeout_ms)
{
    return (HAL_GetTick() - rc_command.last_rx_tick) < timeout_ms;
}

/* Fires once per received byte since we re-arm with size=1 each time. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != UART7) {
        return; /* not our peripheral, ignore */
    }

    switch (rx_state) {
        case RX_WAIT_SYNC1:
            if (rx_byte == SYNC_BYTE_1) {
                frame_buf[0] = rx_byte;
                rx_state = RX_WAIT_SYNC2;
            }
            break;

        case RX_WAIT_SYNC2:
            if (rx_byte == SYNC_BYTE_2) {
                frame_buf[1] = rx_byte;
                frame_idx = 2;
                rx_state = RX_COLLECT_PAYLOAD;
            } else if (rx_byte == SYNC_BYTE_1) {
                /* stay in case this byte is actually sync1 of a new frame */
                frame_buf[0] = rx_byte;
            } else {
                rx_state = RX_WAIT_SYNC1;
            }
            break;

        case RX_COLLECT_PAYLOAD:
            frame_buf[frame_idx++] = rx_byte;
            if (frame_idx >= FRAME_SIZE) {
                uint8_t expected = ComputeChecksum(frame_buf);
                uint8_t received = frame_buf[FRAME_SIZE - 1];
                if (expected == received) {
                    FCFrame_t *f = (FCFrame_t *)frame_buf;
                    rc_command.roll         = f->leftX;
                    rc_command.pitch        = f->leftY;
                    rc_command.yaw          = f->rightX;
                    rc_command.aux          = f->rightY;
                    rc_command.throttle     = f->throttle;
                    rc_command.buttons      = f->buttons;
                    rc_command.last_rx_tick = HAL_GetTick();
                }
                /* Valid or not, reset the state machine for the next frame */
                rx_state  = RX_WAIT_SYNC1;
                frame_idx = 0;
            }
            break;
    }

    /* Re-arm for the next incoming byte */
    HAL_UART_Receive_IT(s_huart, &rx_byte, 1);
}

/* If a framing/noise/overrun error occurs, HAL_UART_Receive_IT silently
 * stops. Without this, one bit of line noise would permanently kill the
 * link until the next reset. Re-arm and resync from scratch. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != UART7) {
        return;
    }
    __HAL_UART_CLEAR_PEFLAG(huart);
    rx_state  = RX_WAIT_SYNC1;
    frame_idx = 0;
    HAL_UART_Receive_IT(s_huart, &rx_byte, 1);
}