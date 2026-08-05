################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ICM42688P.c \
../Core/Src/attitude.c \
../Core/Src/barometre.c \
../Core/Src/bmp280.c \
../Core/Src/filter.c \
../Core/Src/main.c \
../Core/Src/pid.c \
../Core/Src/pwm.c \
../Core/Src/rc_link.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h7xx.c \
../Core/Src/w25q128_ll.c 

OBJS += \
./Core/Src/ICM42688P.o \
./Core/Src/attitude.o \
./Core/Src/barometre.o \
./Core/Src/bmp280.o \
./Core/Src/filter.o \
./Core/Src/main.o \
./Core/Src/pid.o \
./Core/Src/pwm.o \
./Core/Src/rc_link.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h7xx.o \
./Core/Src/w25q128_ll.o 

C_DEPS += \
./Core/Src/ICM42688P.d \
./Core/Src/attitude.d \
./Core/Src/barometre.d \
./Core/Src/bmp280.d \
./Core/Src/filter.d \
./Core/Src/main.d \
./Core/Src/pid.d \
./Core/Src/pwm.d \
./Core/Src/rc_link.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h7xx.d \
./Core/Src/w25q128_ll.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/ICM42688P.cyclo ./Core/Src/ICM42688P.d ./Core/Src/ICM42688P.o ./Core/Src/ICM42688P.su ./Core/Src/attitude.cyclo ./Core/Src/attitude.d ./Core/Src/attitude.o ./Core/Src/attitude.su ./Core/Src/barometre.cyclo ./Core/Src/barometre.d ./Core/Src/barometre.o ./Core/Src/barometre.su ./Core/Src/bmp280.cyclo ./Core/Src/bmp280.d ./Core/Src/bmp280.o ./Core/Src/bmp280.su ./Core/Src/filter.cyclo ./Core/Src/filter.d ./Core/Src/filter.o ./Core/Src/filter.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/pid.cyclo ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/pid.su ./Core/Src/pwm.cyclo ./Core/Src/pwm.d ./Core/Src/pwm.o ./Core/Src/pwm.su ./Core/Src/rc_link.cyclo ./Core/Src/rc_link.d ./Core/Src/rc_link.o ./Core/Src/rc_link.su ./Core/Src/stm32h7xx_hal_msp.cyclo ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_it.cyclo ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h7xx.cyclo ./Core/Src/system_stm32h7xx.d ./Core/Src/system_stm32h7xx.o ./Core/Src/system_stm32h7xx.su ./Core/Src/w25q128_ll.cyclo ./Core/Src/w25q128_ll.d ./Core/Src/w25q128_ll.o ./Core/Src/w25q128_ll.su

.PHONY: clean-Core-2f-Src

