################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../acados/acados/ocp_nlp/ocp_nlp_common.c \
../acados/acados/ocp_nlp/ocp_nlp_constraints_bgh.c \
../acados/acados/ocp_nlp/ocp_nlp_constraints_bgp.c \
../acados/acados/ocp_nlp/ocp_nlp_constraints_common.c \
../acados/acados/ocp_nlp/ocp_nlp_cost_common.c \
../acados/acados/ocp_nlp/ocp_nlp_cost_conl.c \
../acados/acados/ocp_nlp/ocp_nlp_cost_external.c \
../acados/acados/ocp_nlp/ocp_nlp_cost_ls.c \
../acados/acados/ocp_nlp/ocp_nlp_cost_nls.c \
../acados/acados/ocp_nlp/ocp_nlp_ddp.c \
../acados/acados/ocp_nlp/ocp_nlp_dynamics_common.c \
../acados/acados/ocp_nlp/ocp_nlp_dynamics_cont.c \
../acados/acados/ocp_nlp/ocp_nlp_dynamics_disc.c \
../acados/acados/ocp_nlp/ocp_nlp_globalization_common.c \
../acados/acados/ocp_nlp/ocp_nlp_globalization_fixed_step.c \
../acados/acados/ocp_nlp/ocp_nlp_globalization_funnel.c \
../acados/acados/ocp_nlp/ocp_nlp_globalization_merit_backtracking.c \
../acados/acados/ocp_nlp/ocp_nlp_qpscaling.c \
../acados/acados/ocp_nlp/ocp_nlp_reg_common.c \
../acados/acados/ocp_nlp/ocp_nlp_reg_convexify.c \
../acados/acados/ocp_nlp/ocp_nlp_reg_glm.c \
../acados/acados/ocp_nlp/ocp_nlp_reg_mirror.c \
../acados/acados/ocp_nlp/ocp_nlp_reg_noreg.c \
../acados/acados/ocp_nlp/ocp_nlp_reg_project.c \
../acados/acados/ocp_nlp/ocp_nlp_reg_project_reduc_hess.c \
../acados/acados/ocp_nlp/ocp_nlp_sqp.c \
../acados/acados/ocp_nlp/ocp_nlp_sqp_rti.c \
../acados/acados/ocp_nlp/ocp_nlp_sqp_with_feasible_qp.c 

OBJS += \
./acados/acados/ocp_nlp/ocp_nlp_common.o \
./acados/acados/ocp_nlp/ocp_nlp_constraints_bgh.o \
./acados/acados/ocp_nlp/ocp_nlp_constraints_bgp.o \
./acados/acados/ocp_nlp/ocp_nlp_constraints_common.o \
./acados/acados/ocp_nlp/ocp_nlp_cost_common.o \
./acados/acados/ocp_nlp/ocp_nlp_cost_conl.o \
./acados/acados/ocp_nlp/ocp_nlp_cost_external.o \
./acados/acados/ocp_nlp/ocp_nlp_cost_ls.o \
./acados/acados/ocp_nlp/ocp_nlp_cost_nls.o \
./acados/acados/ocp_nlp/ocp_nlp_ddp.o \
./acados/acados/ocp_nlp/ocp_nlp_dynamics_common.o \
./acados/acados/ocp_nlp/ocp_nlp_dynamics_cont.o \
./acados/acados/ocp_nlp/ocp_nlp_dynamics_disc.o \
./acados/acados/ocp_nlp/ocp_nlp_globalization_common.o \
./acados/acados/ocp_nlp/ocp_nlp_globalization_fixed_step.o \
./acados/acados/ocp_nlp/ocp_nlp_globalization_funnel.o \
./acados/acados/ocp_nlp/ocp_nlp_globalization_merit_backtracking.o \
./acados/acados/ocp_nlp/ocp_nlp_qpscaling.o \
./acados/acados/ocp_nlp/ocp_nlp_reg_common.o \
./acados/acados/ocp_nlp/ocp_nlp_reg_convexify.o \
./acados/acados/ocp_nlp/ocp_nlp_reg_glm.o \
./acados/acados/ocp_nlp/ocp_nlp_reg_mirror.o \
./acados/acados/ocp_nlp/ocp_nlp_reg_noreg.o \
./acados/acados/ocp_nlp/ocp_nlp_reg_project.o \
./acados/acados/ocp_nlp/ocp_nlp_reg_project_reduc_hess.o \
./acados/acados/ocp_nlp/ocp_nlp_sqp.o \
./acados/acados/ocp_nlp/ocp_nlp_sqp_rti.o \
./acados/acados/ocp_nlp/ocp_nlp_sqp_with_feasible_qp.o 

C_DEPS += \
./acados/acados/ocp_nlp/ocp_nlp_common.d \
./acados/acados/ocp_nlp/ocp_nlp_constraints_bgh.d \
./acados/acados/ocp_nlp/ocp_nlp_constraints_bgp.d \
./acados/acados/ocp_nlp/ocp_nlp_constraints_common.d \
./acados/acados/ocp_nlp/ocp_nlp_cost_common.d \
./acados/acados/ocp_nlp/ocp_nlp_cost_conl.d \
./acados/acados/ocp_nlp/ocp_nlp_cost_external.d \
./acados/acados/ocp_nlp/ocp_nlp_cost_ls.d \
./acados/acados/ocp_nlp/ocp_nlp_cost_nls.d \
./acados/acados/ocp_nlp/ocp_nlp_ddp.d \
./acados/acados/ocp_nlp/ocp_nlp_dynamics_common.d \
./acados/acados/ocp_nlp/ocp_nlp_dynamics_cont.d \
./acados/acados/ocp_nlp/ocp_nlp_dynamics_disc.d \
./acados/acados/ocp_nlp/ocp_nlp_globalization_common.d \
./acados/acados/ocp_nlp/ocp_nlp_globalization_fixed_step.d \
./acados/acados/ocp_nlp/ocp_nlp_globalization_funnel.d \
./acados/acados/ocp_nlp/ocp_nlp_globalization_merit_backtracking.d \
./acados/acados/ocp_nlp/ocp_nlp_qpscaling.d \
./acados/acados/ocp_nlp/ocp_nlp_reg_common.d \
./acados/acados/ocp_nlp/ocp_nlp_reg_convexify.d \
./acados/acados/ocp_nlp/ocp_nlp_reg_glm.d \
./acados/acados/ocp_nlp/ocp_nlp_reg_mirror.d \
./acados/acados/ocp_nlp/ocp_nlp_reg_noreg.d \
./acados/acados/ocp_nlp/ocp_nlp_reg_project.d \
./acados/acados/ocp_nlp/ocp_nlp_reg_project_reduc_hess.d \
./acados/acados/ocp_nlp/ocp_nlp_sqp.d \
./acados/acados/ocp_nlp/ocp_nlp_sqp_rti.d \
./acados/acados/ocp_nlp/ocp_nlp_sqp_with_feasible_qp.d 


# Each subdirectory must supply rules for building sources it contributes
acados/acados/ocp_nlp/%.o acados/acados/ocp_nlp/%.su acados/acados/ocp_nlp/%.cyclo: ../acados/acados/ocp_nlp/%.c acados/acados/ocp_nlp/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSER_VECT_TAB_ADDRESS -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/lib/blasfeo/include" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/acados/ocp_nlp" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/acados_c" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/blasfeo/include" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/hpipm/include" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/lib/hpipm/include" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/Drivers" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/Drivers/littlefs" -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/Drivers/vl53l1x/platform" -I../Core/Inc -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/acados/lib" -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I"C:/DATA/Projects/Amon_Board/Firmware/Amon_board_firmware-STM32H743/Drivers/vl53l1x/core" -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-acados-2f-acados-2f-ocp_nlp

clean-acados-2f-acados-2f-ocp_nlp:
	-$(RM) ./acados/acados/ocp_nlp/ocp_nlp_common.cyclo ./acados/acados/ocp_nlp/ocp_nlp_common.d ./acados/acados/ocp_nlp/ocp_nlp_common.o ./acados/acados/ocp_nlp/ocp_nlp_common.su ./acados/acados/ocp_nlp/ocp_nlp_constraints_bgh.cyclo ./acados/acados/ocp_nlp/ocp_nlp_constraints_bgh.d ./acados/acados/ocp_nlp/ocp_nlp_constraints_bgh.o ./acados/acados/ocp_nlp/ocp_nlp_constraints_bgh.su ./acados/acados/ocp_nlp/ocp_nlp_constraints_bgp.cyclo ./acados/acados/ocp_nlp/ocp_nlp_constraints_bgp.d ./acados/acados/ocp_nlp/ocp_nlp_constraints_bgp.o ./acados/acados/ocp_nlp/ocp_nlp_constraints_bgp.su ./acados/acados/ocp_nlp/ocp_nlp_constraints_common.cyclo ./acados/acados/ocp_nlp/ocp_nlp_constraints_common.d ./acados/acados/ocp_nlp/ocp_nlp_constraints_common.o ./acados/acados/ocp_nlp/ocp_nlp_constraints_common.su ./acados/acados/ocp_nlp/ocp_nlp_cost_common.cyclo ./acados/acados/ocp_nlp/ocp_nlp_cost_common.d ./acados/acados/ocp_nlp/ocp_nlp_cost_common.o ./acados/acados/ocp_nlp/ocp_nlp_cost_common.su ./acados/acados/ocp_nlp/ocp_nlp_cost_conl.cyclo ./acados/acados/ocp_nlp/ocp_nlp_cost_conl.d ./acados/acados/ocp_nlp/ocp_nlp_cost_conl.o ./acados/acados/ocp_nlp/ocp_nlp_cost_conl.su ./acados/acados/ocp_nlp/ocp_nlp_cost_external.cyclo ./acados/acados/ocp_nlp/ocp_nlp_cost_external.d ./acados/acados/ocp_nlp/ocp_nlp_cost_external.o ./acados/acados/ocp_nlp/ocp_nlp_cost_external.su ./acados/acados/ocp_nlp/ocp_nlp_cost_ls.cyclo ./acados/acados/ocp_nlp/ocp_nlp_cost_ls.d ./acados/acados/ocp_nlp/ocp_nlp_cost_ls.o ./acados/acados/ocp_nlp/ocp_nlp_cost_ls.su ./acados/acados/ocp_nlp/ocp_nlp_cost_nls.cyclo ./acados/acados/ocp_nlp/ocp_nlp_cost_nls.d ./acados/acados/ocp_nlp/ocp_nlp_cost_nls.o ./acados/acados/ocp_nlp/ocp_nlp_cost_nls.su ./acados/acados/ocp_nlp/ocp_nlp_ddp.cyclo ./acados/acados/ocp_nlp/ocp_nlp_ddp.d ./acados/acados/ocp_nlp/ocp_nlp_ddp.o ./acados/acados/ocp_nlp/ocp_nlp_ddp.su ./acados/acados/ocp_nlp/ocp_nlp_dynamics_common.cyclo ./acados/acados/ocp_nlp/ocp_nlp_dynamics_common.d ./acados/acados/ocp_nlp/ocp_nlp_dynamics_common.o ./acados/acados/ocp_nlp/ocp_nlp_dynamics_common.su ./acados/acados/ocp_nlp/ocp_nlp_dynamics_cont.cyclo ./acados/acados/ocp_nlp/ocp_nlp_dynamics_cont.d ./acados/acados/ocp_nlp/ocp_nlp_dynamics_cont.o ./acados/acados/ocp_nlp/ocp_nlp_dynamics_cont.su ./acados/acados/ocp_nlp/ocp_nlp_dynamics_disc.cyclo ./acados/acados/ocp_nlp/ocp_nlp_dynamics_disc.d ./acados/acados/ocp_nlp/ocp_nlp_dynamics_disc.o ./acados/acados/ocp_nlp/ocp_nlp_dynamics_disc.su ./acados/acados/ocp_nlp/ocp_nlp_globalization_common.cyclo ./acados/acados/ocp_nlp/ocp_nlp_globalization_common.d ./acados/acados/ocp_nlp/ocp_nlp_globalization_common.o ./acados/acados/ocp_nlp/ocp_nlp_globalization_common.su ./acados/acados/ocp_nlp/ocp_nlp_globalization_fixed_step.cyclo ./acados/acados/ocp_nlp/ocp_nlp_globalization_fixed_step.d ./acados/acados/ocp_nlp/ocp_nlp_globalization_fixed_step.o ./acados/acados/ocp_nlp/ocp_nlp_globalization_fixed_step.su ./acados/acados/ocp_nlp/ocp_nlp_globalization_funnel.cyclo ./acados/acados/ocp_nlp/ocp_nlp_globalization_funnel.d ./acados/acados/ocp_nlp/ocp_nlp_globalization_funnel.o ./acados/acados/ocp_nlp/ocp_nlp_globalization_funnel.su ./acados/acados/ocp_nlp/ocp_nlp_globalization_merit_backtracking.cyclo ./acados/acados/ocp_nlp/ocp_nlp_globalization_merit_backtracking.d ./acados/acados/ocp_nlp/ocp_nlp_globalization_merit_backtracking.o ./acados/acados/ocp_nlp/ocp_nlp_globalization_merit_backtracking.su ./acados/acados/ocp_nlp/ocp_nlp_qpscaling.cyclo ./acados/acados/ocp_nlp/ocp_nlp_qpscaling.d ./acados/acados/ocp_nlp/ocp_nlp_qpscaling.o ./acados/acados/ocp_nlp/ocp_nlp_qpscaling.su ./acados/acados/ocp_nlp/ocp_nlp_reg_common.cyclo ./acados/acados/ocp_nlp/ocp_nlp_reg_common.d ./acados/acados/ocp_nlp/ocp_nlp_reg_common.o ./acados/acados/ocp_nlp/ocp_nlp_reg_common.su ./acados/acados/ocp_nlp/ocp_nlp_reg_convexify.cyclo ./acados/acados/ocp_nlp/ocp_nlp_reg_convexify.d ./acados/acados/ocp_nlp/ocp_nlp_reg_convexify.o ./acados/acados/ocp_nlp/ocp_nlp_reg_convexify.su ./acados/acados/ocp_nlp/ocp_nlp_reg_glm.cyclo ./acados/acados/ocp_nlp/ocp_nlp_reg_glm.d ./acados/acados/ocp_nlp/ocp_nlp_reg_glm.o ./acados/acados/ocp_nlp/ocp_nlp_reg_glm.su ./acados/acados/ocp_nlp/ocp_nlp_reg_mirror.cyclo ./acados/acados/ocp_nlp/ocp_nlp_reg_mirror.d ./acados/acados/ocp_nlp/ocp_nlp_reg_mirror.o ./acados/acados/ocp_nlp/ocp_nlp_reg_mirror.su ./acados/acados/ocp_nlp/ocp_nlp_reg_noreg.cyclo ./acados/acados/ocp_nlp/ocp_nlp_reg_noreg.d ./acados/acados/ocp_nlp/ocp_nlp_reg_noreg.o ./acados/acados/ocp_nlp/ocp_nlp_reg_noreg.su ./acados/acados/ocp_nlp/ocp_nlp_reg_project.cyclo ./acados/acados/ocp_nlp/ocp_nlp_reg_project.d ./acados/acados/ocp_nlp/ocp_nlp_reg_project.o ./acados/acados/ocp_nlp/ocp_nlp_reg_project.su ./acados/acados/ocp_nlp/ocp_nlp_reg_project_reduc_hess.cyclo ./acados/acados/ocp_nlp/ocp_nlp_reg_project_reduc_hess.d ./acados/acados/ocp_nlp/ocp_nlp_reg_project_reduc_hess.o ./acados/acados/ocp_nlp/ocp_nlp_reg_project_reduc_hess.su ./acados/acados/ocp_nlp/ocp_nlp_sqp.cyclo ./acados/acados/ocp_nlp/ocp_nlp_sqp.d ./acados/acados/ocp_nlp/ocp_nlp_sqp.o ./acados/acados/ocp_nlp/ocp_nlp_sqp.su ./acados/acados/ocp_nlp/ocp_nlp_sqp_rti.cyclo ./acados/acados/ocp_nlp/ocp_nlp_sqp_rti.d ./acados/acados/ocp_nlp/ocp_nlp_sqp_rti.o ./acados/acados/ocp_nlp/ocp_nlp_sqp_rti.su ./acados/acados/ocp_nlp/ocp_nlp_sqp_with_feasible_qp.cyclo ./acados/acados/ocp_nlp/ocp_nlp_sqp_with_feasible_qp.d ./acados/acados/ocp_nlp/ocp_nlp_sqp_with_feasible_qp.o ./acados/acados/ocp_nlp/ocp_nlp_sqp_with_feasible_qp.su

.PHONY: clean-acados-2f-acados-2f-ocp_nlp

