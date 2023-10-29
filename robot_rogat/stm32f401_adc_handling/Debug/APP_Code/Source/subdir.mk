################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../APP_Code/Source/ADC_Handling.c \
../APP_Code/Source/UART_Handling.c 

OBJS += \
./APP_Code/Source/ADC_Handling.o \
./APP_Code/Source/UART_Handling.o 

C_DEPS += \
./APP_Code/Source/ADC_Handling.d \
./APP_Code/Source/UART_Handling.d 


# Each subdirectory must supply rules for building sources it contributes
APP_Code/Source/%.o APP_Code/Source/%.su: ../APP_Code/Source/%.c APP_Code/Source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/RogatGit/ESP32-ROTEM-SELA/robot_rogat/stm32f401_adc_handling/APP_Code/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-APP_Code-2f-Source

clean-APP_Code-2f-Source:
	-$(RM) ./APP_Code/Source/ADC_Handling.d ./APP_Code/Source/ADC_Handling.o ./APP_Code/Source/ADC_Handling.su ./APP_Code/Source/UART_Handling.d ./APP_Code/Source/UART_Handling.o ./APP_Code/Source/UART_Handling.su

.PHONY: clean-APP_Code-2f-Source

