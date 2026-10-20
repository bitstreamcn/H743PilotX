################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../App/Device/Alt/KalmanFilter.c \
../App/Device/Alt/alt.c 

C_DEPS += \
./App/Device/Alt/KalmanFilter.d \
./App/Device/Alt/alt.d 

OBJS += \
./App/Device/Alt/KalmanFilter.o \
./App/Device/Alt/alt.o 


# Each subdirectory must supply rules for building sources it contributes
App/Device/Alt/%.o App/Device/Alt/%.su App/Device/Alt/%.cyclo: ../App/Device/Alt/%.c App/Device/Alt/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_BAND_915 -DUSE_MODEM_LORA -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../App -I../App/Common -I../App/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-App-2f-Device-2f-Alt

clean-App-2f-Device-2f-Alt:
	-$(RM) ./App/Device/Alt/KalmanFilter.cyclo ./App/Device/Alt/KalmanFilter.d ./App/Device/Alt/KalmanFilter.o ./App/Device/Alt/KalmanFilter.su ./App/Device/Alt/alt.cyclo ./App/Device/Alt/alt.d ./App/Device/Alt/alt.o ./App/Device/Alt/alt.su

.PHONY: clean-App-2f-Device-2f-Alt

