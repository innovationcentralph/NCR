################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/sensirion/sensirion_common.c \
../Core/sensirion/sensirion_i2c.c \
../Core/sensirion/sensirion_i2c_hal.c \
../Core/sensirion/sht4x_i2c.c 

OBJS += \
./Core/sensirion/sensirion_common.o \
./Core/sensirion/sensirion_i2c.o \
./Core/sensirion/sensirion_i2c_hal.o \
./Core/sensirion/sht4x_i2c.o 

C_DEPS += \
./Core/sensirion/sensirion_common.d \
./Core/sensirion/sensirion_i2c.d \
./Core/sensirion/sensirion_i2c_hal.d \
./Core/sensirion/sht4x_i2c.d 


# Each subdirectory must supply rules for building sources it contributes
Core/sensirion/%.o Core/sensirion/%.su Core/sensirion/%.cyclo: ../Core/sensirion/%.c Core/sensirion/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Owner/STM32CubeIDE/NCR/NCR_HW_TestCode/Core/LIS2DW12" -I"C:/Users/Owner/STM32CubeIDE/NCR/NCR_HW_TestCode/Core/sht2x" -I"C:/Users/Owner/STM32CubeIDE/NCR/NCR_HW_TestCode/Core/PWXModbus" -I"C:/Users/Owner/STM32CubeIDE/NCR/NCR_HW_TestCode/Core/sensirion" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-sensirion

clean-Core-2f-sensirion:
	-$(RM) ./Core/sensirion/sensirion_common.cyclo ./Core/sensirion/sensirion_common.d ./Core/sensirion/sensirion_common.o ./Core/sensirion/sensirion_common.su ./Core/sensirion/sensirion_i2c.cyclo ./Core/sensirion/sensirion_i2c.d ./Core/sensirion/sensirion_i2c.o ./Core/sensirion/sensirion_i2c.su ./Core/sensirion/sensirion_i2c_hal.cyclo ./Core/sensirion/sensirion_i2c_hal.d ./Core/sensirion/sensirion_i2c_hal.o ./Core/sensirion/sensirion_i2c_hal.su ./Core/sensirion/sht4x_i2c.cyclo ./Core/sensirion/sht4x_i2c.d ./Core/sensirion/sht4x_i2c.o ./Core/sensirion/sht4x_i2c.su

.PHONY: clean-Core-2f-sensirion

