################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/littlefs/lfs.c \
../Drivers/littlefs/lfs_util.c 

C_DEPS += \
./Drivers/littlefs/lfs.d \
./Drivers/littlefs/lfs_util.d 

OBJS += \
./Drivers/littlefs/lfs.o \
./Drivers/littlefs/lfs_util.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/littlefs/%.o Drivers/littlefs/%.su Drivers/littlefs/%.cyclo: ../Drivers/littlefs/%.c Drivers/littlefs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../Drivers/vl53l1x/platform/inc -I../Drivers/vl53l1x/core/inc -I../Drivers/vl53l1x/platform/src -I../Drivers/vl53l1x/core/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-littlefs

clean-Drivers-2f-littlefs:
	-$(RM) ./Drivers/littlefs/lfs.cyclo ./Drivers/littlefs/lfs.d ./Drivers/littlefs/lfs.o ./Drivers/littlefs/lfs.su ./Drivers/littlefs/lfs_util.cyclo ./Drivers/littlefs/lfs_util.d ./Drivers/littlefs/lfs_util.o ./Drivers/littlefs/lfs_util.su

.PHONY: clean-Drivers-2f-littlefs

