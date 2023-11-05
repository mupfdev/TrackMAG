cmake_minimum_required(VERSION 3.15.3)

project(TrackMAG)

enable_language(C ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(SRC_FILES
  Core/Src/app_trackmag.c
  Core/Src/crc.c
  Core/Src/dma.c
  Core/Src/gpio.c
  Core/Src/i2c.c
  Core/Src/main.c
  Core/Src/stm32f1xx_hal_msp.c
  Core/Src/stm32f1xx_hal_timebase_tim.c
  Core/Src/stm32f1xx_it.c
  Core/Src/syscalls.c
  Core/Src/sysmem.c
  Core/Src/system_stm32f1xx.c
  Core/Src/usart.c
  Core/Startup/startup_stm32f103c8tx.s
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_crc.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_i2c.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pcd.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pcd_ex.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_usb.c
  Drivers/lis2mdl-pid/lis2mdl_reg.c
  Drivers/lsm6ds3tr-c-pid/lsm6ds3tr-c_reg.c
  Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Src/usbd_customhid.c
  Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
  Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
  Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
  USB_DEVICE/App/usb_device.c
  USB_DEVICE/App/usbd_custom_hid_if.c
  USB_DEVICE/App/usbd_desc.c
  USB_DEVICE/Target/usbd_conf.c
  )

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T ${CMAKE_CURRENT_SOURCE_DIR}/STM32F103C8TX_FLASH_Release.ld")

add_executable(${PROJECT_NAME} ${SRC_FILES})

target_compile_definitions(${PROJECT_NAME} PRIVATE
  -DUSE_HAL_DRIVER
  -DSTM32F103xB
  )

target_include_directories(${PROJECT_NAME} PRIVATE
  Core/Inc
  Core/Startup
  Drivers/CMSIS/Include
  Drivers/CMSIS/Device/ST/STM32F1xx/Include
  Drivers/STM32F1xx_HAL_Driver/Inc
  Drivers/lis2mdl-pid
  Drivers/lsm6ds3tr-c-pid
  Middlewares/ST/STM32_MotionFX_Library/Inc
  Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Inc
  Middlewares/ST/STM32_USB_Device_Library/Core/Inc
  USB_DEVICE/App
  USB_DEVICE/Target
  )

target_link_libraries(${PROJECT_NAME} PRIVATE c m ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/STM32_MotionFX_Library/Lib/MotionFX_CM3_wc32_ot.a)

set(CORE_FLAGS
  --specs=nano.specs
  --specs=nosys.specs
  -Os -s
  -Wl,-flto
  -Wl,--gc-sections
  -Wl,-Map="TrackMAG.map"
  -mcpu=cortex-m3
  -mfloat-abi=soft
  -mthumb
  -static
  -u _printf_float
  -ffast-math
  )

target_compile_options(${PROJECT_NAME} PRIVATE ${CORE_FLAGS})
target_link_options   (${PROJECT_NAME} PRIVATE ${CORE_FLAGS})

# Print executable size.
add_custom_command(TARGET ${PROJECT_NAME}
  POST_BUILD
  COMMAND arm-none-eabi-size ${PROJECT_NAME}
  )

# Create firmware binary.
add_custom_command(TARGET ${PROJECT_NAME}
  POST_BUILD
  COMMAND arm-none-eabi-objcopy -O binary ${PROJECT_NAME} ${PROJECT_NAME}.bin
  COMMAND python ${CMAKE_SOURCE_DIR}/bootloader/checksum.py ${PROJECT_NAME}.bin
  )