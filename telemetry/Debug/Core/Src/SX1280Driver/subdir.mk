################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/SX1280Driver/SX1280.cpp \
../Core/Src/SX1280Driver/SX1280_hal.cpp 

OBJS += \
./Core/Src/SX1280Driver/SX1280.o \
./Core/Src/SX1280Driver/SX1280_hal.o 

CPP_DEPS += \
./Core/Src/SX1280Driver/SX1280.d \
./Core/Src/SX1280Driver/SX1280_hal.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/SX1280Driver/%.o: ../Core/Src/SX1280Driver/%.cpp Core/Src/SX1280Driver/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m0plus -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G071xx -c -I../Core/Src -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -I../Lib -I../Core/Inc -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-SX1280Driver

clean-Core-2f-Src-2f-SX1280Driver:
	-$(RM) ./Core/Src/SX1280Driver/SX1280.d ./Core/Src/SX1280Driver/SX1280.o ./Core/Src/SX1280Driver/SX1280_hal.d ./Core/Src/SX1280Driver/SX1280_hal.o

.PHONY: clean-Core-2f-Src-2f-SX1280Driver

