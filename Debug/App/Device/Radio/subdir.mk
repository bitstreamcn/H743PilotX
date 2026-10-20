################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../App/Device/Radio/CRSF.c \
../App/Device/Radio/crc2.c \
../App/Device/Radio/crc8.c \
../App/Device/Radio/crsf2.c \
../App/Device/Radio/radio.c \
../App/Device/Radio/sx126x-board.c 

C_DEPS += \
./App/Device/Radio/CRSF.d \
./App/Device/Radio/crc2.d \
./App/Device/Radio/crc8.d \
./App/Device/Radio/crsf2.d \
./App/Device/Radio/radio.d \
./App/Device/Radio/sx126x-board.d 

OBJS += \
./App/Device/Radio/CRSF.o \
./App/Device/Radio/crc2.o \
./App/Device/Radio/crc8.o \
./App/Device/Radio/crsf2.o \
./App/Device/Radio/radio.o \
./App/Device/Radio/sx126x-board.o 


# Each subdirectory must supply rules for building sources it contributes
App/Device/Radio/%.o App/Device/Radio/%.su App/Device/Radio/%.cyclo: ../App/Device/Radio/%.c App/Device/Radio/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_BAND_915 -DUSE_MODEM_LORA -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../App -I../App/Common -I../App/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-App-2f-Device-2f-Radio

clean-App-2f-Device-2f-Radio:
	-$(RM) ./App/Device/Radio/CRSF.cyclo ./App/Device/Radio/CRSF.d ./App/Device/Radio/CRSF.o ./App/Device/Radio/CRSF.su ./App/Device/Radio/crc2.cyclo ./App/Device/Radio/crc2.d ./App/Device/Radio/crc2.o ./App/Device/Radio/crc2.su ./App/Device/Radio/crc8.cyclo ./App/Device/Radio/crc8.d ./App/Device/Radio/crc8.o ./App/Device/Radio/crc8.su ./App/Device/Radio/crsf2.cyclo ./App/Device/Radio/crsf2.d ./App/Device/Radio/crsf2.o ./App/Device/Radio/crsf2.su ./App/Device/Radio/radio.cyclo ./App/Device/Radio/radio.d ./App/Device/Radio/radio.o ./App/Device/Radio/radio.su ./App/Device/Radio/sx126x-board.cyclo ./App/Device/Radio/sx126x-board.d ./App/Device/Radio/sx126x-board.o ./App/Device/Radio/sx126x-board.su

.PHONY: clean-App-2f-Device-2f-Radio

