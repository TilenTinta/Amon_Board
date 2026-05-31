################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../acados/acados/utils/external_function_generic.c \
../acados/acados/utils/math.c \
../acados/acados/utils/mem.c \
../acados/acados/utils/print.c \
../acados/acados/utils/timing.c 

OBJS += \
./acados/acados/utils/external_function_generic.o \
./acados/acados/utils/math.o \
./acados/acados/utils/mem.o \
./acados/acados/utils/print.o \
./acados/acados/utils/timing.o 

C_DEPS += \
./acados/acados/utils/external_function_generic.d \
./acados/acados/utils/math.d \
./acados/acados/utils/mem.d \
./acados/acados/utils/print.d \
./acados/acados/utils/timing.d 


# Each subdirectory must supply rules for building sources it contributes
acados/acados/utils/%.o acados/acados/utils/%.su acados/acados/utils/%.cyclo: ../acados/acados/utils/%.c acados/acados/utils/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSER_VECT_TAB_ADDRESS -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/lib/blasfeo/include" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/acados/ocp_nlp" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/acados_c" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/blasfeo/include" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/hpipm/include" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/lib/hpipm/include" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/Drivers" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/Drivers/littlefs" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/Drivers/vl53l1x/platform" -I../Core/Inc -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/lib" -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/Drivers/vl53l1x/core" -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-acados-2f-acados-2f-utils

clean-acados-2f-acados-2f-utils:
	-$(RM) ./acados/acados/utils/external_function_generic.cyclo ./acados/acados/utils/external_function_generic.d ./acados/acados/utils/external_function_generic.o ./acados/acados/utils/external_function_generic.su ./acados/acados/utils/math.cyclo ./acados/acados/utils/math.d ./acados/acados/utils/math.o ./acados/acados/utils/math.su ./acados/acados/utils/mem.cyclo ./acados/acados/utils/mem.d ./acados/acados/utils/mem.o ./acados/acados/utils/mem.su ./acados/acados/utils/print.cyclo ./acados/acados/utils/print.d ./acados/acados/utils/print.o ./acados/acados/utils/print.su ./acados/acados/utils/timing.cyclo ./acados/acados/utils/timing.d ./acados/acados/utils/timing.o ./acados/acados/utils/timing.su

.PHONY: clean-acados-2f-acados-2f-utils

