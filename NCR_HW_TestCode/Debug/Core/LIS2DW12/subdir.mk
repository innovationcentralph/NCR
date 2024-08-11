################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/LIS2DW12/lis2dw12_reg.c 

OBJS += \
./Core/LIS2DW12/lis2dw12_reg.o 

C_DEPS += \
./Core/LIS2DW12/lis2dw12_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Core/LIS2DW12/%.o Core/LIS2DW12/%.su Core/LIS2DW12/%.cyclo: ../Core/LIS2DW12/%.c Core/LIS2DW12/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Owner/STM32CubeIDE/NCR/NCR_HW_TestCode/Core/LIS2DW12" -I"C:/Users/Owner/STM32CubeIDE/NCR/NCR_HW_TestCode/Core/sht4x" -I"C:/Users/Owner/STM32CubeIDE/NCR/NCR_HW_TestCode/Core/sht2x" -I"C:/Users/Owner/STM32CubeIDE/NCR/NCR_HW_TestCode/Core/PWXModbus" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-LIS2DW12

clean-Core-2f-LIS2DW12:
	-$(RM) ./Core/LIS2DW12/lis2dw12_reg.cyclo ./Core/LIS2DW12/lis2dw12_reg.d ./Core/LIS2DW12/lis2dw12_reg.o ./Core/LIS2DW12/lis2dw12_reg.su

.PHONY: clean-Core-2f-LIS2DW12

