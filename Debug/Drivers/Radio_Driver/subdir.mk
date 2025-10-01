################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Radio_Driver/radio_board_if.c \
../Drivers/Radio_Driver/radio_driver.c 

OBJS += \
./Drivers/Radio_Driver/radio_board_if.o \
./Drivers/Radio_Driver/radio_driver.o 

C_DEPS += \
./Drivers/Radio_Driver/radio_board_if.d \
./Drivers/Radio_Driver/radio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Radio_Driver/%.o Drivers/Radio_Driver/%.su Drivers/Radio_Driver/%.cyclo: ../Drivers/Radio_Driver/%.c Drivers/Radio_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WL55xx -c -I../Core/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Drivers/CMSIS/Include -I"C:/Users/mehna/Documents/STM32 workspace/STM32_SubGHz_LoRa_Messenger/Drivers/BSP/STM32WLxx_Nucleo" -I"C:/Users/mehna/Documents/STM32 workspace/STM32_SubGHz_LoRa_Messenger/Utils/misc" -I"C:/Users/mehna/Documents/STM32 workspace/STM32_SubGHz_LoRa_Messenger/Utils/conf" -I"C:/Users/mehna/Documents/STM32 workspace/STM32_SubGHz_LoRa_Messenger/Drivers/Radio_Driver" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-Radio_Driver

clean-Drivers-2f-Radio_Driver:
	-$(RM) ./Drivers/Radio_Driver/radio_board_if.cyclo ./Drivers/Radio_Driver/radio_board_if.d ./Drivers/Radio_Driver/radio_board_if.o ./Drivers/Radio_Driver/radio_board_if.su ./Drivers/Radio_Driver/radio_driver.cyclo ./Drivers/Radio_Driver/radio_driver.d ./Drivers/Radio_Driver/radio_driver.o ./Drivers/Radio_Driver/radio_driver.su

.PHONY: clean-Drivers-2f-Radio_Driver

