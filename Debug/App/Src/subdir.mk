################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../App/Src/CRSFRadio.cpp \
../App/Src/Servo.cpp \
../App/Src/application.cpp \
../App/Src/bmp280.cpp \
../App/Src/bottom.cpp \
../App/Src/car.cpp \
../App/Src/common.cpp \
../App/Src/configlink.cpp \
../App/Src/controller.cpp \
../App/Src/cppmain.cpp \
../App/Src/file.cpp \
../App/Src/flow_kalman.cpp \
../App/Src/gpiox.cpp \
../App/Src/gps.cpp \
../App/Src/i2cx.cpp \
../App/Src/imu_ex.cpp \
../App/Src/led.cpp \
../App/Src/log.cpp \
../App/Src/mag_calibration.cpp \
../App/Src/pid.cpp \
../App/Src/sensor.cpp \
../App/Src/syscalls.cpp \
../App/Src/timx.cpp \
../App/Src/uartx.cpp 

C_SRCS += \
../App/Src/Config.c 

C_DEPS += \
./App/Src/Config.d 

OBJS += \
./App/Src/CRSFRadio.o \
./App/Src/Config.o \
./App/Src/Servo.o \
./App/Src/application.o \
./App/Src/bmp280.o \
./App/Src/bottom.o \
./App/Src/car.o \
./App/Src/common.o \
./App/Src/configlink.o \
./App/Src/controller.o \
./App/Src/cppmain.o \
./App/Src/file.o \
./App/Src/flow_kalman.o \
./App/Src/gpiox.o \
./App/Src/gps.o \
./App/Src/i2cx.o \
./App/Src/imu_ex.o \
./App/Src/led.o \
./App/Src/log.o \
./App/Src/mag_calibration.o \
./App/Src/pid.o \
./App/Src/sensor.o \
./App/Src/syscalls.o \
./App/Src/timx.o \
./App/Src/uartx.o 

CPP_DEPS += \
./App/Src/CRSFRadio.d \
./App/Src/Servo.d \
./App/Src/application.d \
./App/Src/bmp280.d \
./App/Src/bottom.d \
./App/Src/car.d \
./App/Src/common.d \
./App/Src/configlink.d \
./App/Src/controller.d \
./App/Src/cppmain.d \
./App/Src/file.d \
./App/Src/flow_kalman.d \
./App/Src/gpiox.d \
./App/Src/gps.d \
./App/Src/i2cx.d \
./App/Src/imu_ex.d \
./App/Src/led.d \
./App/Src/log.d \
./App/Src/mag_calibration.d \
./App/Src/pid.d \
./App/Src/sensor.d \
./App/Src/syscalls.d \
./App/Src/timx.d \
./App/Src/uartx.d 


# Each subdirectory must supply rules for building sources it contributes
App/Src/%.o App/Src/%.su App/Src/%.cyclo: ../App/Src/%.cpp App/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_BAND_915 -DUSE_MODEM_LORA -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../App -I../App/Common -I../App/Inc -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
App/Src/%.o App/Src/%.su App/Src/%.cyclo: ../App/Src/%.c App/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_BAND_915 -DUSE_MODEM_LORA -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../App -I../App/Common -I../App/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-App-2f-Src

clean-App-2f-Src:
	-$(RM) ./App/Src/CRSFRadio.cyclo ./App/Src/CRSFRadio.d ./App/Src/CRSFRadio.o ./App/Src/CRSFRadio.su ./App/Src/Config.cyclo ./App/Src/Config.d ./App/Src/Config.o ./App/Src/Config.su ./App/Src/Servo.cyclo ./App/Src/Servo.d ./App/Src/Servo.o ./App/Src/Servo.su ./App/Src/application.cyclo ./App/Src/application.d ./App/Src/application.o ./App/Src/application.su ./App/Src/bmp280.cyclo ./App/Src/bmp280.d ./App/Src/bmp280.o ./App/Src/bmp280.su ./App/Src/bottom.cyclo ./App/Src/bottom.d ./App/Src/bottom.o ./App/Src/bottom.su ./App/Src/car.cyclo ./App/Src/car.d ./App/Src/car.o ./App/Src/car.su ./App/Src/common.cyclo ./App/Src/common.d ./App/Src/common.o ./App/Src/common.su ./App/Src/configlink.cyclo ./App/Src/configlink.d ./App/Src/configlink.o ./App/Src/configlink.su ./App/Src/controller.cyclo ./App/Src/controller.d ./App/Src/controller.o ./App/Src/controller.su ./App/Src/cppmain.cyclo ./App/Src/cppmain.d ./App/Src/cppmain.o ./App/Src/cppmain.su ./App/Src/file.cyclo ./App/Src/file.d ./App/Src/file.o ./App/Src/file.su ./App/Src/flow_kalman.cyclo ./App/Src/flow_kalman.d ./App/Src/flow_kalman.o ./App/Src/flow_kalman.su ./App/Src/gpiox.cyclo ./App/Src/gpiox.d ./App/Src/gpiox.o ./App/Src/gpiox.su ./App/Src/gps.cyclo ./App/Src/gps.d ./App/Src/gps.o ./App/Src/gps.su ./App/Src/i2cx.cyclo ./App/Src/i2cx.d ./App/Src/i2cx.o ./App/Src/i2cx.su ./App/Src/imu_ex.cyclo ./App/Src/imu_ex.d ./App/Src/imu_ex.o ./App/Src/imu_ex.su ./App/Src/led.cyclo ./App/Src/led.d ./App/Src/led.o ./App/Src/led.su ./App/Src/log.cyclo ./App/Src/log.d ./App/Src/log.o ./App/Src/log.su ./App/Src/mag_calibration.cyclo ./App/Src/mag_calibration.d ./App/Src/mag_calibration.o ./App/Src/mag_calibration.su ./App/Src/pid.cyclo ./App/Src/pid.d ./App/Src/pid.o ./App/Src/pid.su ./App/Src/sensor.cyclo ./App/Src/sensor.d ./App/Src/sensor.o ./App/Src/sensor.su ./App/Src/syscalls.cyclo ./App/Src/syscalls.d ./App/Src/syscalls.o ./App/Src/syscalls.su ./App/Src/timx.cyclo ./App/Src/timx.d ./App/Src/timx.o ./App/Src/timx.su ./App/Src/uartx.cyclo ./App/Src/uartx.d ./App/Src/uartx.o ./App/Src/uartx.su

.PHONY: clean-App-2f-Src

