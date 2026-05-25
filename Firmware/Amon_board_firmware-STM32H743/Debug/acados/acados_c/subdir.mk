################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../acados/acados_c/condensing_interface.c \
../acados/acados_c/dense_qp_interface.c \
../acados/acados_c/external_function_interface.c \
../acados/acados_c/ocp_nlp_interface.c \
../acados/acados_c/ocp_qp_interface.c \
../acados/acados_c/sim_interface.c 

OBJS += \
./acados/acados_c/condensing_interface.o \
./acados/acados_c/dense_qp_interface.o \
./acados/acados_c/external_function_interface.o \
./acados/acados_c/ocp_nlp_interface.o \
./acados/acados_c/ocp_qp_interface.o \
./acados/acados_c/sim_interface.o 

C_DEPS += \
./acados/acados_c/condensing_interface.d \
./acados/acados_c/dense_qp_interface.d \
./acados/acados_c/external_function_interface.d \
./acados/acados_c/ocp_nlp_interface.d \
./acados/acados_c/ocp_qp_interface.d \
./acados/acados_c/sim_interface.d 


# Each subdirectory must supply rules for building sources it contributes
acados/acados_c/%.o acados/acados_c/%.su acados/acados_c/%.cyclo: ../acados/acados_c/%.c acados/acados_c/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/lib/blasfeo/include" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/acados/ocp_nlp" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/acados_c" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/blasfeo/include" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/hpipm/include" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/lib/hpipm/include" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/Drivers" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/Drivers/littlefs" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/Drivers/vl53l1x/platform" -I../Core/Inc -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/lib" -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/Drivers/vl53l1x/core" -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-acados-2f-acados_c

clean-acados-2f-acados_c:
	-$(RM) ./acados/acados_c/condensing_interface.cyclo ./acados/acados_c/condensing_interface.d ./acados/acados_c/condensing_interface.o ./acados/acados_c/condensing_interface.su ./acados/acados_c/dense_qp_interface.cyclo ./acados/acados_c/dense_qp_interface.d ./acados/acados_c/dense_qp_interface.o ./acados/acados_c/dense_qp_interface.su ./acados/acados_c/external_function_interface.cyclo ./acados/acados_c/external_function_interface.d ./acados/acados_c/external_function_interface.o ./acados/acados_c/external_function_interface.su ./acados/acados_c/ocp_nlp_interface.cyclo ./acados/acados_c/ocp_nlp_interface.d ./acados/acados_c/ocp_nlp_interface.o ./acados/acados_c/ocp_nlp_interface.su ./acados/acados_c/ocp_qp_interface.cyclo ./acados/acados_c/ocp_qp_interface.d ./acados/acados_c/ocp_qp_interface.o ./acados/acados_c/ocp_qp_interface.su ./acados/acados_c/sim_interface.cyclo ./acados/acados_c/sim_interface.d ./acados/acados_c/sim_interface.o ./acados/acados_c/sim_interface.su

.PHONY: clean-acados-2f-acados_c

