################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/ETHERNET/socket.c \
../Drivers/ETHERNET/wizchip_conf.c 

C_DEPS += \
./Drivers/ETHERNET/socket.d \
./Drivers/ETHERNET/wizchip_conf.d 

OBJS += \
./Drivers/ETHERNET/socket.o \
./Drivers/ETHERNET/wizchip_conf.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/ETHERNET/%.o Drivers/ETHERNET/%.su Drivers/ETHERNET/%.cyclo: ../Drivers/ETHERNET/%.c Drivers/ETHERNET/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu18 -DUSE_HAL_DRIVER -DSTM32H723xx -DUSE_PWR_LDO_SUPPLY -c -I../Core/Inc -I../Drivers/ETHERNET -I../Drivers/ETHERNET/W5500 -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/ARM/DSP/Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-ETHERNET

clean-Drivers-2f-ETHERNET:
	-$(RM) ./Drivers/ETHERNET/socket.cyclo ./Drivers/ETHERNET/socket.d ./Drivers/ETHERNET/socket.o ./Drivers/ETHERNET/socket.su ./Drivers/ETHERNET/wizchip_conf.cyclo ./Drivers/ETHERNET/wizchip_conf.d ./Drivers/ETHERNET/wizchip_conf.o ./Drivers/ETHERNET/wizchip_conf.su

.PHONY: clean-Drivers-2f-ETHERNET

