#include "pwm.h"
extern TIM_HandleTypeDef htim5;




uint16_t computePower(uint16_t us_command) {

    if (us_command < 1000) us_command = 1000; 
    if (us_command > 2000) us_command = 2000; 

    return us_command; 
}


void setMotor1(uint16_t us_command) {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, computePower(us_command)); 
}

void setMotor2(uint16_t us_command) {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, computePower(us_command));
}

void setMotor3(uint16_t us_command) {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, computePower(us_command));
}

void setMotor4(uint16_t us_command) {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, computePower(us_command));
}