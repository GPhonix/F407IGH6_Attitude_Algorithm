################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../USER/IST8310/IST8310.c 

OBJS += \
./USER/IST8310/IST8310.o 

C_DEPS += \
./USER/IST8310/IST8310.d 


# Each subdirectory must supply rules for building sources it contributes
USER/IST8310/%.o USER/IST8310/%.su: ../USER/IST8310/%.c USER/IST8310/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/data/Project/CUBEIDE_project/F407IGH6_Attitude _Algorithm/USER/bsp_RGB" -I"D:/data/Project/CUBEIDE_project/F407IGH6_Attitude _Algorithm/USER/bsp_ChassisControl" -I"D:/data/Project/CUBEIDE_project/F407IGH6_Attitude _Algorithm/USER/bsp_RC" -I"D:/data/Project/CUBEIDE_project/F407IGH6_Attitude _Algorithm/USER/myCAN" -I"D:/data/Project/CUBEIDE_project/F407IGH6_Attitude _Algorithm/USER/PID_primary" -I"D:/data/Project/CUBEIDE_project/F407IGH6_Attitude _Algorithm/USER/OLED" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-USER-2f-IST8310

clean-USER-2f-IST8310:
	-$(RM) ./USER/IST8310/IST8310.d ./USER/IST8310/IST8310.o ./USER/IST8310/IST8310.su

.PHONY: clean-USER-2f-IST8310

