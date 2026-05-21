################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/vl53l1x/core/VL53L1X_api.c \
../Drivers/vl53l1x/core/VL53L1X_calibration.c 

OBJS += \
./Drivers/vl53l1x/core/VL53L1X_api.o \
./Drivers/vl53l1x/core/VL53L1X_calibration.o 

C_DEPS += \
./Drivers/vl53l1x/core/VL53L1X_api.d \
./Drivers/vl53l1x/core/VL53L1X_calibration.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/vl53l1x/core/%.o Drivers/vl53l1x/core/%.su Drivers/vl53l1x/core/%.cyclo: ../Drivers/vl53l1x/core/%.c Drivers/vl53l1x/core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/lib/blasfeo/include" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/lib/hpipm/include" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/Drivers" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/acados" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/Drivers/littlefs" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/Drivers/vl53l1x/platform" -I../Core/Inc -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/lib" -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/Drivers/vl53l1x/core" -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-vl53l1x-2f-core

clean-Drivers-2f-vl53l1x-2f-core:
	-$(RM) ./Drivers/vl53l1x/core/VL53L1X_api.cyclo ./Drivers/vl53l1x/core/VL53L1X_api.d ./Drivers/vl53l1x/core/VL53L1X_api.o ./Drivers/vl53l1x/core/VL53L1X_api.su ./Drivers/vl53l1x/core/VL53L1X_calibration.cyclo ./Drivers/vl53l1x/core/VL53L1X_calibration.d ./Drivers/vl53l1x/core/VL53L1X_calibration.o ./Drivers/vl53l1x/core/VL53L1X_calibration.su

.PHONY: clean-Drivers-2f-vl53l1x-2f-core

