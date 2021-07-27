################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/NL/Desktop/sensorbox/Middlewares/Third_Party/FatFs/src/option/syscall.c \
C:/Users/NL/Desktop/sensorbox/Middlewares/Third_Party/FatFs/src/option/unicode.c 

OBJS += \
./Middlewares/FatFs/Options/syscall.o \
./Middlewares/FatFs/Options/unicode.o 

C_DEPS += \
./Middlewares/FatFs/Options/syscall.d \
./Middlewares/FatFs/Options/unicode.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/FatFs/Options/syscall.o: C:/Users/NL/Desktop/sensorbox/Middlewares/Third_Party/FatFs/src/option/syscall.c Middlewares/FatFs/Options/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DSTM32L4R9xx -DUSE_HAL_DRIVER -DUSE_USB_FS -DUSE_SENSORTILEBOX -DARM_MATH_CM4 -c -I../../../../Inc -I../../../../../Drivers/CMSIS/Include -I../../../../../Drivers/CMSIS/DSP/Include -I../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../Drivers/BSP/SensorTile.box -I../../../../../Drivers/BSP/Components/Common -I../../../../../Drivers/BSP/Components/hts221 -I../../../../../Drivers/BSP/Components/lps22hh -I../../../../../Drivers/BSP/Components/lsm6dsox -I../../../../../Drivers/BSP/Components/lis2mdl -I../../../../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../../../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../../../../Middlewares/ST/STM32_MotionAC_Library/Inc -I../../../../../Middlewares/ST/STM32_MotionAW_Library/Inc -I../../../../../Middlewares/ST/STM32_MotionEC_Library/Inc -I../../../../../Middlewares/ST/STM32_MotionFX_Library/Inc -I../../../../../Middlewares/ST/STM32_MotionGC_Library/Inc -I../../../../../Middlewares/ST/STM32_MotionID_Library/Inc -I../../../../../Middlewares/ST/STM32_MotionMC_Library/Inc -I../../../../../Middlewares/ST/STM32_MotionPM_Library/Inc -I../../../../../Middlewares/ST/STM32_MotionPW_Library/Inc -I../../../../../Middlewares/ST/STM32_MotionTL_Library/Inc -I../../../../../Middlewares/Third_Party/FatFs/src -O3 -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/FatFs/Options/syscall.d" -MT"$@"  -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Middlewares/FatFs/Options/unicode.o: C:/Users/NL/Desktop/sensorbox/Middlewares/Third_Party/FatFs/src/option/unicode.c Middlewares/FatFs/Options/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DSTM32L4R9xx -DUSE_HAL_DRIVER -DUSE_USB_FS -DUSE_SENSORTILEBOX -DARM_MATH_CM4 -c -I../../../../Inc -I../../../../../Drivers/CMSIS/Include -I../../../../../Drivers/CMSIS/DSP/Include -I../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../Drivers/BSP/SensorTile.box -I../../../../../Drivers/BSP/Components/Common -I../../../../../Drivers/BSP/Components/hts221 -I../../../../../Drivers/BSP/Components/lps22hh -I../../../../../Drivers/BSP/Components/lsm6dsox -I../../../../../Drivers/BSP/Components/lis2mdl -I../../../../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../../../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../../../../Middlewares/ST/STM32_MotionAC_Library/Inc -I../../../../../Middlewares/ST/STM32_MotionAW_Library/Inc -I../../../../../Middlewares/ST/STM32_MotionEC_Library/Inc -I../../../../../Middlewares/ST/STM32_MotionFX_Library/Inc -I../../../../../Middlewares/ST/STM32_MotionGC_Library/Inc -I../../../../../Middlewares/ST/STM32_MotionID_Library/Inc -I../../../../../Middlewares/ST/STM32_MotionMC_Library/Inc -I../../../../../Middlewares/ST/STM32_MotionPM_Library/Inc -I../../../../../Middlewares/ST/STM32_MotionPW_Library/Inc -I../../../../../Middlewares/ST/STM32_MotionTL_Library/Inc -I../../../../../Middlewares/Third_Party/FatFs/src -O3 -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/FatFs/Options/unicode.d" -MT"$@"  -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"

