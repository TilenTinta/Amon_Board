################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/AMPC.c \
../Core/Src/BME280.c \
../Core/Src/GNSS.c \
../Core/Src/HMC5883L.c \
../Core/Src/MPU6050.c \
../Core/Src/NMPC.c \
../Core/Src/NRF24L01.c \
../Core/Src/PMW3901.c \
../Core/Src/PWM.c \
../Core/Src/autopilot.c \
../Core/Src/data_transcode.c \
../Core/Src/drone_data.c \
../Core/Src/filters.c \
../Core/Src/flash.c \
../Core/Src/flash_W25QXXX.c \
../Core/Src/logging.c \
../Core/Src/main.c \
../Core/Src/nmpc_platform.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h7xx.c 

OBJS += \
./Core/Src/AMPC.o \
./Core/Src/BME280.o \
./Core/Src/GNSS.o \
./Core/Src/HMC5883L.o \
./Core/Src/MPU6050.o \
./Core/Src/NMPC.o \
./Core/Src/NRF24L01.o \
./Core/Src/PMW3901.o \
./Core/Src/PWM.o \
./Core/Src/autopilot.o \
./Core/Src/data_transcode.o \
./Core/Src/drone_data.o \
./Core/Src/filters.o \
./Core/Src/flash.o \
./Core/Src/flash_W25QXXX.o \
./Core/Src/logging.o \
./Core/Src/main.o \
./Core/Src/nmpc_platform.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h7xx.o 

C_DEPS += \
./Core/Src/AMPC.d \
./Core/Src/BME280.d \
./Core/Src/GNSS.d \
./Core/Src/HMC5883L.d \
./Core/Src/MPU6050.d \
./Core/Src/NMPC.d \
./Core/Src/NRF24L01.d \
./Core/Src/PMW3901.d \
./Core/Src/PWM.d \
./Core/Src/autopilot.d \
./Core/Src/data_transcode.d \
./Core/Src/drone_data.d \
./Core/Src/filters.d \
./Core/Src/flash.d \
./Core/Src/flash_W25QXXX.d \
./Core/Src/logging.d \
./Core/Src/main.d \
./Core/Src/nmpc_platform.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h7xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSER_VECT_TAB_ADDRESS -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -DLA_REFERENCE -DTARGET_GENERIC -DACADOS_WITH_STATIC_MEMORY -DEXT_DEP -c -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/lib/blasfeo/lib" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/acados/ocp_nlp" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/acados_c" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/blasfeo/include" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/hpipm/include" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/lib/hpipm/lib" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/Drivers" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/Drivers/littlefs" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/Drivers/vl53l1x/platform" -I../Core/Inc -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/lib" -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/Drivers/vl53l1x/core" -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/AMPC.cyclo ./Core/Src/AMPC.d ./Core/Src/AMPC.o ./Core/Src/AMPC.su ./Core/Src/BME280.cyclo ./Core/Src/BME280.d ./Core/Src/BME280.o ./Core/Src/BME280.su ./Core/Src/GNSS.cyclo ./Core/Src/GNSS.d ./Core/Src/GNSS.o ./Core/Src/GNSS.su ./Core/Src/HMC5883L.cyclo ./Core/Src/HMC5883L.d ./Core/Src/HMC5883L.o ./Core/Src/HMC5883L.su ./Core/Src/MPU6050.cyclo ./Core/Src/MPU6050.d ./Core/Src/MPU6050.o ./Core/Src/MPU6050.su ./Core/Src/NMPC.cyclo ./Core/Src/NMPC.d ./Core/Src/NMPC.o ./Core/Src/NMPC.su ./Core/Src/NRF24L01.cyclo ./Core/Src/NRF24L01.d ./Core/Src/NRF24L01.o ./Core/Src/NRF24L01.su ./Core/Src/PMW3901.cyclo ./Core/Src/PMW3901.d ./Core/Src/PMW3901.o ./Core/Src/PMW3901.su ./Core/Src/PWM.cyclo ./Core/Src/PWM.d ./Core/Src/PWM.o ./Core/Src/PWM.su ./Core/Src/autopilot.cyclo ./Core/Src/autopilot.d ./Core/Src/autopilot.o ./Core/Src/autopilot.su ./Core/Src/data_transcode.cyclo ./Core/Src/data_transcode.d ./Core/Src/data_transcode.o ./Core/Src/data_transcode.su ./Core/Src/drone_data.cyclo ./Core/Src/drone_data.d ./Core/Src/drone_data.o ./Core/Src/drone_data.su ./Core/Src/filters.cyclo ./Core/Src/filters.d ./Core/Src/filters.o ./Core/Src/filters.su ./Core/Src/flash.cyclo ./Core/Src/flash.d ./Core/Src/flash.o ./Core/Src/flash.su ./Core/Src/flash_W25QXXX.cyclo ./Core/Src/flash_W25QXXX.d ./Core/Src/flash_W25QXXX.o ./Core/Src/flash_W25QXXX.su ./Core/Src/logging.cyclo ./Core/Src/logging.d ./Core/Src/logging.o ./Core/Src/logging.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/nmpc_platform.cyclo ./Core/Src/nmpc_platform.d ./Core/Src/nmpc_platform.o ./Core/Src/nmpc_platform.su ./Core/Src/stm32h7xx_hal_msp.cyclo ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_it.cyclo ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h7xx.cyclo ./Core/Src/system_stm32h7xx.d ./Core/Src/system_stm32h7xx.o ./Core/Src/system_stm32h7xx.su

.PHONY: clean-Core-2f-Src

