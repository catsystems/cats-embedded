################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/Transmission/Transmission.cpp 

OBJS += \
./Core/Src/Transmission/Transmission.o 

CPP_DEPS += \
./Core/Src/Transmission/Transmission.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Transmission/%.o: ../Core/Src/Transmission/%.cpp Core/Src/Transmission/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m0plus -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G071xx -c -I../Core/Src -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -I../Lib -I../Core/Inc -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Transmission

clean-Core-2f-Src-2f-Transmission:
	-$(RM) ./Core/Src/Transmission/Transmission.d ./Core/Src/Transmission/Transmission.o

.PHONY: clean-Core-2f-Src-2f-Transmission

