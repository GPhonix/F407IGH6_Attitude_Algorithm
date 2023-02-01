################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User_F407IGH6/OLED/OLED.c 

OBJS += \
./User_F407IGH6/OLED/OLED.o 

C_DEPS += \
./User_F407IGH6/OLED/OLED.d 


# Each subdirectory must supply rules for building sources it contributes
User_F407IGH6/OLED/%.o User_F407IGH6/OLED/%.su: ../User_F407IGH6/OLED/%.c User_F407IGH6/OLED/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/data/Project/CUBEIDE_project/F407IGH6_Attitude_Algorithm/User_F407IGH6/Algorithms_Lib" -I"D:/data/Project/CUBEIDE_project/F407IGH6_Attitude_Algorithm/User_F407IGH6/IMU_C" -I"D:/data/Project/CUBEIDE_project/F407IGH6_Attitude_Algorithm/User_F407IGH6/BMI088" -I"D:/data/Project/CUBEIDE_project/F407IGH6_Attitude_Algorithm/User_F407IGH6/LED_F407IGH6TR" -I"D:/data/Project/CUBEIDE_project/F407IGH6_Attitude_Algorithm/User_F407IGH6/IST8310" -I"D:/data/Project/CUBEIDE_project/F407IGH6_Attitude_Algorithm/User_F407IGH6/OLED" -I"D:/data/Project/CUBEIDE_project/F407IGH6_Attitude_Algorithm/User_F407IGH6/PID_primary" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-User_F407IGH6-2f-OLED

clean-User_F407IGH6-2f-OLED:
	-$(RM) ./User_F407IGH6/OLED/OLED.d ./User_F407IGH6/OLED/OLED.o ./User_F407IGH6/OLED/OLED.su

.PHONY: clean-User_F407IGH6-2f-OLED

