################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/FHSS/FHSS.cpp \
../Core/Src/FHSS/crc.cpp \
../Core/Src/FHSS/random.cpp 

OBJS += \
./Core/Src/FHSS/FHSS.o \
./Core/Src/FHSS/crc.o \
./Core/Src/FHSS/random.o 

CPP_DEPS += \
./Core/Src/FHSS/FHSS.d \
./Core/Src/FHSS/crc.d \
./Core/Src/FHSS/random.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/FHSS/%.o: ../Core/Src/FHSS/%.cpp Core/Src/FHSS/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m0plus -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G071xx -c -I../Core/Src -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -I../Lib -I../Core/Inc -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-FHSS

clean-Core-2f-Src-2f-FHSS:
	-$(RM) ./Core/Src/FHSS/FHSS.d ./Core/Src/FHSS/FHSS.o ./Core/Src/FHSS/crc.d ./Core/Src/FHSS/crc.o ./Core/Src/FHSS/random.d ./Core/Src/FHSS/random.o

.PHONY: clean-Core-2f-Src-2f-FHSS

