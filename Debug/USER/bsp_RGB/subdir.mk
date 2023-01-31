################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../USER/bsp_RGB/RGB.c 

OBJS += \
./USER/bsp_RGB/RGB.o 

C_DEPS += \
./USER/bsp_RGB/RGB.d 


# Each subdirectory must supply rules for building sources it contributes
USER/bsp_RGB/%.o USER/bsp_RGB/%.su: ../USER/bsp_RGB/%.c USER/bsp_RGB/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/data/Project/CUBEIDE_project/F407IGH6_Attitude _Algorithm/USER/bsp_RGB" -I"D:/data/Project/CUBEIDE_project/F407IGH6_Attitude _Algorithm/USER/bsp_ChassisControl" -I"D:/data/Project/CUBEIDE_project/F407IGH6_Attitude _Algorithm/USER/bsp_RC" -I"D:/data/Project/CUBEIDE_project/F407IGH6_Attitude _Algorithm/USER/myCAN" -I"D:/data/Project/CUBEIDE_project/F407IGH6_Attitude _Algorithm/USER/PID_primary" -I"D:/data/Project/CUBEIDE_project/F407IGH6_Attitude _Algorithm/USER/OLED" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-USER-2f-bsp_RGB

clean-USER-2f-bsp_RGB:
	-$(RM) ./USER/bsp_RGB/RGB.d ./USER/bsp_RGB/RGB.o ./USER/bsp_RGB/RGB.su

.PHONY: clean-USER-2f-bsp_RGB

