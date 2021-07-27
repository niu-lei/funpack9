################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/STM32_USBD_Library/Core/subdir.mk
-include Middlewares/STM32_USBD_Library/Class/CDC/subdir.mk
-include Middlewares/FatFs/Options/subdir.mk
-include Middlewares/FatFs/Core/subdir.mk
-include Example/User/subdir.mk
-include Example/SW4STM32/subdir.mk
-include Example/AlgoBuilder/subdir.mk
-include Drivers/STM32L4xx_HAL_Driver/subdir.mk
-include Drivers/CMSIS/subdir.mk
-include Drivers/BSP/SensorTile.box/subdir.mk
-include Drivers/BSP/Components/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := SensorTile.box-Project
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX := 
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME).$(BUILD_ARTIFACT_EXTENSION)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
SensorTile.box-Project.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
SensorTile.box-Project.list \

OBJCOPY_BIN += \
SensorTile.box-Project.bin \


# All Target
all:
	+@$(MAKE) --no-print-directory main-build && $(MAKE) --no-print-directory post-build

# Main-build Target
main-build: SensorTile.box-Project.elf secondary-outputs

# Tool invocations
SensorTile.box-Project.elf: $(OBJS) $(USER_OBJS) ../LinkerScript.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "SensorTile.box-Project.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"../LinkerScript.ld" -Wl,-Map="SensorTile.box-Project.map" -Wl,--gc-sections -static -Wl,--start-group -l:MotionAC_CM4F_wc32_ot.a -l:MotionAW_CM4F_wc32_ot.a -l:MotionEC_CM4F_wc32_ot.a -l:MotionFX_CM4F_wc32_ot.a -l:MotionGC_CM4F_wc32_ot.a -l:MotionID_CM4F_wc32_ot.a -l:MotionMC_CM4F_wc32_ot.a -l:MotionPM_CM4F_wc32_ot.a -l:MotionPW_CM4F_wc32_ot.a -l:MotionTL_CM4F_wc32_ot.a -larm_cortexM4l_math -lm -lc -Wl,--end-group -L../../../../../Middlewares/ST/STM32_MotionAC_Library/Lib -L../../../../../Middlewares/ST/STM32_MotionAW_Library/Lib -L../../../../../Middlewares/ST/STM32_MotionEC_Library/Lib -L../../../../../Middlewares/ST/STM32_MotionFX_Library/Lib -L../../../../../Middlewares/ST/STM32_MotionGC_Library/Lib -L../../../../../Middlewares/ST/STM32_MotionID_Library/Lib -L../../../../../Middlewares/ST/STM32_MotionMC_Library/Lib -L../../../../../Middlewares/ST/STM32_MotionPM_Library/Lib -L../../../../../Middlewares/ST/STM32_MotionPW_Library/Lib -L../../../../../Middlewares/ST/STM32_MotionTL_Library/Lib -L../../../../../Drivers/CMSIS/Lib/GCC  -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -u _printf_float -u _scanf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

SensorTile.box-Project.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "SensorTile.box-Project.list"
	@echo 'Finished building: $@'
	@echo ' '

SensorTile.box-Project.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "SensorTile.box-Project.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

post-build:
	arm-none-eabi-objcopy -O binary "SensorTile.box-Project.elf" "SensorTile.box-Project.bin"
	arm-none-eabi-objcopy -O ihex "SensorTile.box-Project.elf" "SensorTile.box-Project.hex"
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents fail-specified-linker-script-missing warn-no-linker-script-specified
.SECONDARY: post-build

-include ../makefile.targets
