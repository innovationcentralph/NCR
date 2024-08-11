################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/sht4x/sht40.c 

OBJS += \
./Core/sht4x/sht40.o 

C_DEPS += \
./Core/sht4x/sht40.d 


# Each subdirectory must supply rules for building sources it contributes
Core/sht4x/%.o Core/sht4x/%.su Core/sht4x/%.cyclo: ../Core/sht4x/%.c Core/sht4x/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Owner/STM32CubeIDE/NCR/NCR_HW_TestCode/Core/LIS2DW12" -I"C:/Users/Owner/STM32CubeIDE/NCR/NCR_HW_TestCode/Core/sht4x" -I"C:/Users/Owner/STM32CubeIDE/NCR/NCR_HW_TestCode/Core/sht2x" -I"C:/Users/Owner/STM32CubeIDE/NCR/NCR_HW_TestCode/Core/PWXModbus" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-sht4x

clean-Core-2f-sht4x:
	-$(RM) ./Core/sht4x/sht40.cyclo ./Core/sht4x/sht40.d ./Core/sht4x/sht40.o ./Core/sht4x/sht40.su

.PHONY: clean-Core-2f-sht4x

