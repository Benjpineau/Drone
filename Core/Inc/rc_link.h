#ifndef RC_LINK_H
#define RC_LINK_H

#include "main.h"
#include <stdint.h>

/* Latest decoded command from the ESP32 controller link.
 * Field ranges match what the ESP32 sends:
 *   roll/pitch/yaw/aux : approx -511..512 (raw Bluepad32 stick axes)
 *   throttle           : 0..255
 *   buttons            : bit0=A, bit1=B, bit2=X, bit3=Y
 * last_rx_tick is HAL_GetTick() at the moment the last VALID (checksum-passed)
 * frame was decoded -- use RC_IsLinkValid() to check for signal loss.
 */
typedef struct {
    int16_t  roll;
    int16_t  pitch;
    int16_t  yaw;
    int16_t  aux;
    uint8_t  throttle;
    uint8_t  buttons;
    uint32_t last_rx_tick;
} RC_Command_t;

extern volatile RC_Command_t rc_command;

/* Call once after MX_UART7_Init(). Arms the first byte-receive interrupt. */
void RC_Init(UART_HandleTypeDef *huart);

/* Returns 1 if a valid frame was received within the last timeout_ms, else 0.
 * Call this every loop iteration and force throttle to zero / disarm if it
 * returns 0 -- this is your failsafe against a dead Bluetooth link, and it
 * does NOT depend on the ESP32 successfully sending its own failsafe frame. */
uint8_t RC_IsLinkValid(uint32_t timeout_ms);

#endif /* RC_LINK_H */
