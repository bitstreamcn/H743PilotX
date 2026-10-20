################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../App/Common/kalman.cpp 

C_SRCS += \
../App/Common/ring_buffer.c 

C_DEPS += \
./App/Common/ring_buffer.d 

OBJS += \
./App/Common/kalman.o \
./App/Common/ring_buffer.o 

CPP_DEPS += \
./App/Common/kalman.d 


# Each subdirectory must supply rules for building sources it contributes
App/Common/%.o App/Common/%.su App/Common/%.cyclo: ../App/Common/%.cpp App/Common/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_BAND_915 -DUSE_MODEM_LORA -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../App -I../App/Common -I../App/Inc -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
App/Common/%.o App/Common/%.su App/Common/%.cyclo: ../App/Common/%.c App/Common/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_BAND_915 -DUSE_MODEM_LORA -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../App -I../App/Common -I../App/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-App-2f-Common

clean-App-2f-Common:
	-$(RM) ./App/Common/kalman.cyclo ./App/Common/kalman.d ./App/Common/kalman.o ./App/Common/kalman.su ./App/Common/ring_buffer.cyclo ./App/Common/ring_buffer.d ./App/Common/ring_buffer.o ./App/Common/ring_buffer.su

.PHONY: clean-App-2f-Common

