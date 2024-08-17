################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/sht2x/sht2x_for_stm32_hal.c 

OBJS += \
./Core/sht2x/sht2x_for_stm32_hal.o 

C_DEPS += \
./Core/sht2x/sht2x_for_stm32_hal.d 


# Each subdirectory must supply rules for building sources it contributes
Core/sht2x/%.o Core/sht2x/%.su Core/sht2x/%.cyclo: ../Core/sht2x/%.c Core/sht2x/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Owner/OneDrive/Documents/GitHub/NCR_Pulled/NCR_HW_TestCode/Core/LIS2DW12" -I"C:/Users/Owner/OneDrive/Documents/GitHub/NCR_Pulled/NCR_HW_TestCode/Core/sht2x" -I"C:/Users/Owner/OneDrive/Documents/GitHub/NCR_Pulled/NCR_HW_TestCode/Core/PWXModbus" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-sht2x

clean-Core-2f-sht2x:
	-$(RM) ./Core/sht2x/sht2x_for_stm32_hal.cyclo ./Core/sht2x/sht2x_for_stm32_hal.d ./Core/sht2x/sht2x_for_stm32_hal.o ./Core/sht2x/sht2x_for_stm32_hal.su

.PHONY: clean-Core-2f-sht2x

