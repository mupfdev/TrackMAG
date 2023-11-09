################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Src/usbd_customhid.c 

OBJS += \
./Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Src/usbd_customhid.o 

C_DEPS += \
./Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Src/usbd_customhid.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Src/%.o Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Src/%.su Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Src/%.cyclo: ../Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Src/%.c Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Inc -I../Middlewares/ST/STM32_MotionFX_Library/Inc -I"C:/Development/Projects/TrackMAG/firmware/Drivers/lis2mdl-pid" -I"C:/Development/Projects/TrackMAG/firmware/Drivers/lsm6ds3tr-c-pid" -Os -ffunction-sections -fdata-sections -Wall -pedantic -pedantic-errors -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Middlewares-2f-ST-2f-STM32_USB_Device_Library-2f-Class-2f-CustomHID-2f-Src

clean-Middlewares-2f-ST-2f-STM32_USB_Device_Library-2f-Class-2f-CustomHID-2f-Src:
	-$(RM) ./Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Src/usbd_customhid.cyclo ./Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Src/usbd_customhid.d ./Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Src/usbd_customhid.o ./Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Src/usbd_customhid.su

.PHONY: clean-Middlewares-2f-ST-2f-STM32_USB_Device_Library-2f-Class-2f-CustomHID-2f-Src

