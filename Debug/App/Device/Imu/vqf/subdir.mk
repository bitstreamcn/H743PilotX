################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../App/Device/Imu/vqf/vqf.c 

C_DEPS += \
./App/Device/Imu/vqf/vqf.d 

OBJS += \
./App/Device/Imu/vqf/vqf.o 


# Each subdirectory must supply rules for building sources it contributes
App/Device/Imu/vqf/%.o App/Device/Imu/vqf/%.su App/Device/Imu/vqf/%.cyclo: ../App/Device/Imu/vqf/%.c App/Device/Imu/vqf/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_BAND_915 -DUSE_MODEM_LORA -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../App -I../App/Common -I../App/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-App-2f-Device-2f-Imu-2f-vqf

clean-App-2f-Device-2f-Imu-2f-vqf:
	-$(RM) ./App/Device/Imu/vqf/vqf.cyclo ./App/Device/Imu/vqf/vqf.d ./App/Device/Imu/vqf/vqf.o ./App/Device/Imu/vqf/vqf.su

.PHONY: clean-App-2f-Device-2f-Imu-2f-vqf

