################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/GPS/gps.c 

C_DEPS += \
./Core/Src/GPS/gps.d 

OBJS += \
./Core/Src/GPS/gps.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/GPS/%.o: ../Core/Src/GPS/%.c Core/Src/GPS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G071xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-GPS

clean-Core-2f-Src-2f-GPS:
	-$(RM) ./Core/Src/GPS/gps.d ./Core/Src/GPS/gps.o

.PHONY: clean-Core-2f-Src-2f-GPS

