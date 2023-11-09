################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/lis2mdl-pid/lis2mdl_reg.c 

OBJS += \
./Drivers/lis2mdl-pid/lis2mdl_reg.o 

C_DEPS += \
./Drivers/lis2mdl-pid/lis2mdl_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/lis2mdl-pid/%.o Drivers/lis2mdl-pid/%.su Drivers/lis2mdl-pid/%.cyclo: ../Drivers/lis2mdl-pid/%.c Drivers/lis2mdl-pid/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Inc -I../Middlewares/ST/STM32_MotionFX_Library/Inc -I"C:/Development/Projects/TrackMAG/firmware/Drivers/lis2mdl-pid" -I"C:/Development/Projects/TrackMAG/firmware/Drivers/lsm6ds3tr-c-pid" -Os -ffunction-sections -fdata-sections -Wall -pedantic -pedantic-errors -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-lis2mdl-2d-pid

clean-Drivers-2f-lis2mdl-2d-pid:
	-$(RM) ./Drivers/lis2mdl-pid/lis2mdl_reg.cyclo ./Drivers/lis2mdl-pid/lis2mdl_reg.d ./Drivers/lis2mdl-pid/lis2mdl_reg.o ./Drivers/lis2mdl-pid/lis2mdl_reg.su

.PHONY: clean-Drivers-2f-lis2mdl-2d-pid

