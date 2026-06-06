################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../acados/acados/sim/sim_collocation_utils.c \
../acados/acados/sim/sim_common.c \
../acados/acados/sim/sim_erk_integrator.c \
../acados/acados/sim/sim_gnsf.c \
../acados/acados/sim/sim_irk_integrator.c \
../acados/acados/sim/sim_lifted_irk_integrator.c 

OBJS += \
./acados/acados/sim/sim_collocation_utils.o \
./acados/acados/sim/sim_common.o \
./acados/acados/sim/sim_erk_integrator.o \
./acados/acados/sim/sim_gnsf.o \
./acados/acados/sim/sim_irk_integrator.o \
./acados/acados/sim/sim_lifted_irk_integrator.o 

C_DEPS += \
./acados/acados/sim/sim_collocation_utils.d \
./acados/acados/sim/sim_common.d \
./acados/acados/sim/sim_erk_integrator.d \
./acados/acados/sim/sim_gnsf.d \
./acados/acados/sim/sim_irk_integrator.d \
./acados/acados/sim/sim_lifted_irk_integrator.d 


# Each subdirectory must supply rules for building sources it contributes
acados/acados/sim/%.o acados/acados/sim/%.su acados/acados/sim/%.cyclo: ../acados/acados/sim/%.c acados/acados/sim/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSER_VECT_TAB_ADDRESS -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/lib/blasfeo/include" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/acados/ocp_nlp" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/acados_c" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/blasfeo/include" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/hpipm/include" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/lib/hpipm/include" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/Drivers" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/Drivers/littlefs" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/Drivers/vl53l1x/platform" -I../Core/Inc -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/lib" -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/Drivers/vl53l1x/core" -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-acados-2f-acados-2f-sim

clean-acados-2f-acados-2f-sim:
	-$(RM) ./acados/acados/sim/sim_collocation_utils.cyclo ./acados/acados/sim/sim_collocation_utils.d ./acados/acados/sim/sim_collocation_utils.o ./acados/acados/sim/sim_collocation_utils.su ./acados/acados/sim/sim_common.cyclo ./acados/acados/sim/sim_common.d ./acados/acados/sim/sim_common.o ./acados/acados/sim/sim_common.su ./acados/acados/sim/sim_erk_integrator.cyclo ./acados/acados/sim/sim_erk_integrator.d ./acados/acados/sim/sim_erk_integrator.o ./acados/acados/sim/sim_erk_integrator.su ./acados/acados/sim/sim_gnsf.cyclo ./acados/acados/sim/sim_gnsf.d ./acados/acados/sim/sim_gnsf.o ./acados/acados/sim/sim_gnsf.su ./acados/acados/sim/sim_irk_integrator.cyclo ./acados/acados/sim/sim_irk_integrator.d ./acados/acados/sim/sim_irk_integrator.o ./acados/acados/sim/sim_irk_integrator.su ./acados/acados/sim/sim_lifted_irk_integrator.cyclo ./acados/acados/sim/sim_lifted_irk_integrator.d ./acados/acados/sim/sim_lifted_irk_integrator.o ./acados/acados/sim/sim_lifted_irk_integrator.su

.PHONY: clean-acados-2f-acados-2f-sim

