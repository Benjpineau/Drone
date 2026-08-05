#ifndef PWM_H
#define PWM_H

#include "stm32h7xx_hal.h"

uint16_t computePower(uint16_t us_command);

void setMotor1(uint16_t us_command);
void setMotor2(uint16_t us_command);
void setMotor3(uint16_t us_command);
void setMotor4(uint16_t us_command);

#endif //PWM_H_