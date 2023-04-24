################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/PS2/PS2.c 

OBJS += \
./Core/Inc/PS2/PS2.o 

C_DEPS += \
./Core/Inc/PS2/PS2.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/PS2/%.o Core/Inc/PS2/%.su: ../Core/Inc/PS2/%.c Core/Inc/PS2/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-PS2

clean-Core-2f-Inc-2f-PS2:
	-$(RM) ./Core/Inc/PS2/PS2.d ./Core/Inc/PS2/PS2.o ./Core/Inc/PS2/PS2.su

.PHONY: clean-Core-2f-Inc-2f-PS2

