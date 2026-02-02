################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/BME280.c \
../Core/Src/GNSS.c \
../Core/Src/MPU6050.c \
../Core/Src/NRF24L01.c \
../Core/Src/PWM.c \
../Core/Src/data_transcode.c \
../Core/Src/drone_data.c \
../Core/Src/filters.c \
../Core/Src/main.c \
../Core/Src/regulators.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

C_DEPS += \
./Core/Src/BME280.d \
./Core/Src/GNSS.d \
./Core/Src/MPU6050.d \
./Core/Src/NRF24L01.d \
./Core/Src/PWM.d \
./Core/Src/data_transcode.d \
./Core/Src/drone_data.d \
./Core/Src/filters.d \
./Core/Src/main.d \
./Core/Src/regulators.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 

OBJS += \
./Core/Src/BME280.o \
./Core/Src/GNSS.o \
./Core/Src/MPU6050.o \
./Core/Src/NRF24L01.o \
./Core/Src/PWM.o \
./Core/Src/data_transcode.o \
./Core/Src/drone_data.o \
./Core/Src/filters.o \
./Core/Src/main.o \
./Core/Src/regulators.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-MCU/Drivers/vl53l1x/core" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-MCU/Drivers/vl53l1x/platform" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-MCU/Drivers" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-MCU/Drivers/vl53l1x" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/BME280.cyclo ./Core/Src/BME280.d ./Core/Src/BME280.o ./Core/Src/BME280.su ./Core/Src/GNSS.cyclo ./Core/Src/GNSS.d ./Core/Src/GNSS.o ./Core/Src/GNSS.su ./Core/Src/MPU6050.cyclo ./Core/Src/MPU6050.d ./Core/Src/MPU6050.o ./Core/Src/MPU6050.su ./Core/Src/NRF24L01.cyclo ./Core/Src/NRF24L01.d ./Core/Src/NRF24L01.o ./Core/Src/NRF24L01.su ./Core/Src/PWM.cyclo ./Core/Src/PWM.d ./Core/Src/PWM.o ./Core/Src/PWM.su ./Core/Src/data_transcode.cyclo ./Core/Src/data_transcode.d ./Core/Src/data_transcode.o ./Core/Src/data_transcode.su ./Core/Src/drone_data.cyclo ./Core/Src/drone_data.d ./Core/Src/drone_data.o ./Core/Src/drone_data.su ./Core/Src/filters.cyclo ./Core/Src/filters.d ./Core/Src/filters.o ./Core/Src/filters.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/regulators.cyclo ./Core/Src/regulators.d ./Core/Src/regulators.o ./Core/Src/regulators.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

