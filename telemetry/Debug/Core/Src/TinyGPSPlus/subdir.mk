################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/TinyGPSPlus/TinyGPS.cpp 

OBJS += \
./Core/Src/TinyGPSPlus/TinyGPS.o 

CPP_DEPS += \
./Core/Src/TinyGPSPlus/TinyGPS.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/TinyGPSPlus/%.o: ../Core/Src/TinyGPSPlus/%.cpp Core/Src/TinyGPSPlus/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m0plus -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G071xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -I../Lib -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-TinyGPSPlus

clean-Core-2f-Src-2f-TinyGPSPlus:
	-$(RM) ./Core/Src/TinyGPSPlus/TinyGPS.d ./Core/Src/TinyGPSPlus/TinyGPS.o

.PHONY: clean-Core-2f-Src-2f-TinyGPSPlus

