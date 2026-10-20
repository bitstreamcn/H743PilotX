################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../App/Device/Bmp/bmp5xx.cpp 

C_SRCS += \
../App/Device/Bmp/bmp5.c 

C_DEPS += \
./App/Device/Bmp/bmp5.d 

OBJS += \
./App/Device/Bmp/bmp5.o \
./App/Device/Bmp/bmp5xx.o 

CPP_DEPS += \
./App/Device/Bmp/bmp5xx.d 


# Each subdirectory must supply rules for building sources it contributes
App/Device/Bmp/%.o App/Device/Bmp/%.su App/Device/Bmp/%.cyclo: ../App/Device/Bmp/%.c App/Device/Bmp/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_BAND_915 -DUSE_MODEM_LORA -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../App -I../App/Common -I../App/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
App/Device/Bmp/%.o App/Device/Bmp/%.su App/Device/Bmp/%.cyclo: ../App/Device/Bmp/%.cpp App/Device/Bmp/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_BAND_915 -DUSE_MODEM_LORA -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../App -I../App/Common -I../App/Inc -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-App-2f-Device-2f-Bmp

clean-App-2f-Device-2f-Bmp:
	-$(RM) ./App/Device/Bmp/bmp5.cyclo ./App/Device/Bmp/bmp5.d ./App/Device/Bmp/bmp5.o ./App/Device/Bmp/bmp5.su ./App/Device/Bmp/bmp5xx.cyclo ./App/Device/Bmp/bmp5xx.d ./App/Device/Bmp/bmp5xx.o ./App/Device/Bmp/bmp5xx.su

.PHONY: clean-App-2f-Device-2f-Bmp

