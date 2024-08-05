################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/SHT4x/sht4x.c \
../Core/SHT4x/sht4x_example_usage.c 

OBJS += \
./Core/SHT4x/sht4x.o \
./Core/SHT4x/sht4x_example_usage.o 

C_DEPS += \
./Core/SHT4x/sht4x.d \
./Core/SHT4x/sht4x_example_usage.d 


# Each subdirectory must supply rules for building sources it contributes
Core/SHT4x/%.o Core/SHT4x/%.su Core/SHT4x/%.cyclo: ../Core/SHT4x/%.c Core/SHT4x/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Owner/STM32CubeIDE/NCR/NCR_HW_TestCode/Core/SHT4x" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-SHT4x

clean-Core-2f-SHT4x:
	-$(RM) ./Core/SHT4x/sht4x.cyclo ./Core/SHT4x/sht4x.d ./Core/SHT4x/sht4x.o ./Core/SHT4x/sht4x.su ./Core/SHT4x/sht4x_example_usage.cyclo ./Core/SHT4x/sht4x_example_usage.d ./Core/SHT4x/sht4x_example_usage.o ./Core/SHT4x/sht4x_example_usage.su

.PHONY: clean-Core-2f-SHT4x

