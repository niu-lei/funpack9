################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
C:/Users/NL/Desktop/sensorbox/Project/CubeIDE/STM32L4R9ZI-SensorTile.box/startup_stm32l4r9xx.s 

C_SRCS += \
C:/Users/NL/Desktop/sensorbox/Project/CubeIDE/STM32L4R9ZI-SensorTile.box/syscalls.c 

OBJS += \
./Example/SW4STM32/startup_stm32l4r9xx.o \
./Example/SW4STM32/syscalls.o 

S_DEPS += \
./Example/SW4STM32/startup_stm32l4r9xx.d 

C_DEPS += \
./Example/SW4STM32/syscalls.d 


# Each subdirectory must supply rules for building sources it contributes
Example/SW4STM32/startup_stm32l4r9xx.o: C:/Users/NL/Desktop/sensorbox/Project/CubeIDE/STM32L4R9ZI-SensorTile.box/startup_stm32l4r9xx.s Example/SW4STM32/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -c -x assembler-with-cpp -MMD -MP -MF"Example/SW4STM32/startup_stm32l4r9xx.d" -MT"$@"  -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@" "$<"
Example/SW4STM32/syscalls.o: C:/Users/NL/Desktop/sensorbox/Project/CubeIDE/STM32L4R9ZI-SensorTile.box/syscalls.c Example/SW4STM32/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DSTM32L4R9xx -DUSE_HAL_DRIVER -DUSE_USB_FS -DUSE_SENSORTILEBOX -DARM_MATH_CM4 -c -I../../../../Inc -I../../../../../Drivers/CMSIS/Include -I../../../../../Drivers/CMSIS/DSP/Include -I../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../Drivers/BSP/SensorTile.box -I../../../../../Drivers/BSP/Components/Common -I../../../../../Drivers/BSP/Components/hts221 -I../../../../../Drivers/BSP/Components/lps22hh -I../../../../../Drivers/BSP/Components/lsm6dsox -I../../../../../Drivers/BSP/Components/lis2mdl -I../../../../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../../../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../../../../Middlewares/ST/STM32_MotionAC_Library/Inc -I../../../../../Middlewares/ST/STM32_MotionAW_Library/Inc -I../../../../../Middlewares/ST/STM32_MotionEC_Library/Inc -I../../../../../Middlewares/ST/STM32_MotionFX_Library/Inc -I../../../../../Middlewares/ST/STM32_MotionGC_Library/Inc -I../../../../../Middlewares/ST/STM32_MotionID_Library/Inc -I../../../../../Middlewares/ST/STM32_MotionMC_Library/Inc -I../../../../../Middlewares/ST/STM32_MotionPM_Library/Inc -I../../../../../Middlewares/ST/STM32_MotionPW_Library/Inc -I../../../../../Middlewares/ST/STM32_MotionTL_Library/Inc -I../../../../../Middlewares/Third_Party/FatFs/src -O3 -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"Example/SW4STM32/syscalls.d" -MT"$@"  -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"

