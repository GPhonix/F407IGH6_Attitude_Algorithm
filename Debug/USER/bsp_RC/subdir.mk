################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../USER/bsp_RC/rc_uart.c 

OBJS += \
./USER/bsp_RC/rc_uart.o 

C_DEPS += \
./USER/bsp_RC/rc_uart.d 


# Each subdirectory must supply rules for building sources it contributes
USER/bsp_RC/%.o USER/bsp_RC/%.su: ../USER/bsp_RC/%.c USER/bsp_RC/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/data/Project/CUBEIDE_project/F407IGH6_Attitude _Algorithm/USER/bsp_RGB" -I"D:/data/Project/CUBEIDE_project/F407IGH6_Attitude _Algorithm/USER/bsp_ChassisControl" -I"D:/data/Project/CUBEIDE_project/F407IGH6_Attitude _Algorithm/USER/bsp_RC" -I"D:/data/Project/CUBEIDE_project/F407IGH6_Attitude _Algorithm/USER/myCAN" -I"D:/data/Project/CUBEIDE_project/F407IGH6_Attitude _Algorithm/USER/PID_primary" -I"D:/data/Project/CUBEIDE_project/F407IGH6_Attitude _Algorithm/USER/OLED" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-USER-2f-bsp_RC

clean-USER-2f-bsp_RC:
	-$(RM) ./USER/bsp_RC/rc_uart.d ./USER/bsp_RC/rc_uart.o ./USER/bsp_RC/rc_uart.su

.PHONY: clean-USER-2f-bsp_RC

