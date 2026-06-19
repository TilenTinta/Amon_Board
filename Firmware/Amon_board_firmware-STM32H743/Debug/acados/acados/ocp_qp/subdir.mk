################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../acados/acados/ocp_qp/ocp_qp_common.c \
../acados/acados/ocp_qp/ocp_qp_common_frontend.c \
../acados/acados/ocp_qp/ocp_qp_full_condensing.c \
../acados/acados/ocp_qp/ocp_qp_hpipm.c \
../acados/acados/ocp_qp/ocp_qp_partial_condensing.c \
../acados/acados/ocp_qp/ocp_qp_xcond_solver.c 

OBJS += \
./acados/acados/ocp_qp/ocp_qp_common.o \
./acados/acados/ocp_qp/ocp_qp_common_frontend.o \
./acados/acados/ocp_qp/ocp_qp_full_condensing.o \
./acados/acados/ocp_qp/ocp_qp_hpipm.o \
./acados/acados/ocp_qp/ocp_qp_partial_condensing.o \
./acados/acados/ocp_qp/ocp_qp_xcond_solver.o 

C_DEPS += \
./acados/acados/ocp_qp/ocp_qp_common.d \
./acados/acados/ocp_qp/ocp_qp_common_frontend.d \
./acados/acados/ocp_qp/ocp_qp_full_condensing.d \
./acados/acados/ocp_qp/ocp_qp_hpipm.d \
./acados/acados/ocp_qp/ocp_qp_partial_condensing.d \
./acados/acados/ocp_qp/ocp_qp_xcond_solver.d 


# Each subdirectory must supply rules for building sources it contributes
acados/acados/ocp_qp/%.o acados/acados/ocp_qp/%.su acados/acados/ocp_qp/%.cyclo: ../acados/acados/ocp_qp/%.c acados/acados/ocp_qp/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSER_VECT_TAB_ADDRESS -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -DLA_REFERENCE -DTARGET_GENERIC -DACADOS_WITH_STATIC_MEMORY -DEXT_DEP -c -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/lib/blasfeo/lib" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/acados/ocp_nlp" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/acados_c" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/blasfeo/include" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/hpipm/include" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/lib/hpipm/lib" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/Drivers" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/Drivers/littlefs" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/Drivers/vl53l1x/platform" -I../Core/Inc -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/lib" -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/Drivers/vl53l1x/core" -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-acados-2f-acados-2f-ocp_qp

clean-acados-2f-acados-2f-ocp_qp:
	-$(RM) ./acados/acados/ocp_qp/ocp_qp_common.cyclo ./acados/acados/ocp_qp/ocp_qp_common.d ./acados/acados/ocp_qp/ocp_qp_common.o ./acados/acados/ocp_qp/ocp_qp_common.su ./acados/acados/ocp_qp/ocp_qp_common_frontend.cyclo ./acados/acados/ocp_qp/ocp_qp_common_frontend.d ./acados/acados/ocp_qp/ocp_qp_common_frontend.o ./acados/acados/ocp_qp/ocp_qp_common_frontend.su ./acados/acados/ocp_qp/ocp_qp_full_condensing.cyclo ./acados/acados/ocp_qp/ocp_qp_full_condensing.d ./acados/acados/ocp_qp/ocp_qp_full_condensing.o ./acados/acados/ocp_qp/ocp_qp_full_condensing.su ./acados/acados/ocp_qp/ocp_qp_hpipm.cyclo ./acados/acados/ocp_qp/ocp_qp_hpipm.d ./acados/acados/ocp_qp/ocp_qp_hpipm.o ./acados/acados/ocp_qp/ocp_qp_hpipm.su ./acados/acados/ocp_qp/ocp_qp_partial_condensing.cyclo ./acados/acados/ocp_qp/ocp_qp_partial_condensing.d ./acados/acados/ocp_qp/ocp_qp_partial_condensing.o ./acados/acados/ocp_qp/ocp_qp_partial_condensing.su ./acados/acados/ocp_qp/ocp_qp_xcond_solver.cyclo ./acados/acados/ocp_qp/ocp_qp_xcond_solver.d ./acados/acados/ocp_qp/ocp_qp_xcond_solver.o ./acados/acados/ocp_qp/ocp_qp_xcond_solver.su

.PHONY: clean-acados-2f-acados-2f-ocp_qp

