################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../acados/acados/dense_qp/dense_qp_common.c \
../acados/acados/dense_qp/dense_qp_hpipm.c 

OBJS += \
./acados/acados/dense_qp/dense_qp_common.o \
./acados/acados/dense_qp/dense_qp_hpipm.o 

C_DEPS += \
./acados/acados/dense_qp/dense_qp_common.d \
./acados/acados/dense_qp/dense_qp_hpipm.d 


# Each subdirectory must supply rules for building sources it contributes
acados/acados/dense_qp/%.o acados/acados/dense_qp/%.su acados/acados/dense_qp/%.cyclo: ../acados/acados/dense_qp/%.c acados/acados/dense_qp/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSER_VECT_TAB_ADDRESS -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/lib/blasfeo/include" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/acados/ocp_nlp" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/acados_c" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/blasfeo/include" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/hpipm/include" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/lib/hpipm/include" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/Drivers" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/Drivers/littlefs" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/Drivers/vl53l1x/platform" -I../Core/Inc -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/lib" -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/Drivers/vl53l1x/core" -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-acados-2f-acados-2f-dense_qp

clean-acados-2f-acados-2f-dense_qp:
	-$(RM) ./acados/acados/dense_qp/dense_qp_common.cyclo ./acados/acados/dense_qp/dense_qp_common.d ./acados/acados/dense_qp/dense_qp_common.o ./acados/acados/dense_qp/dense_qp_common.su ./acados/acados/dense_qp/dense_qp_hpipm.cyclo ./acados/acados/dense_qp/dense_qp_hpipm.d ./acados/acados/dense_qp/dense_qp_hpipm.o ./acados/acados/dense_qp/dense_qp_hpipm.su

.PHONY: clean-acados-2f-acados-2f-dense_qp

