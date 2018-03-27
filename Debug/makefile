################################################################################
# Automatically-generated file. Do not edit!
################################################################################

CG_TOOL_ROOT := /home/chel/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.6.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./tm4c123gh6pm_startup_ccs.obj" \
"./LCD/lcd.obj" \
"./Receiver/receiver.obj" \
"./Servo/servo.obj" \
"./Servo/utilFunc.obj" \
"./Transmitter/main.obj" \
"./Transmitter/transmitter.obj" \
"../tm4c123gh6pm.cmd" \
$(GEN_CMDS__FLAG) \
-llibc.a \
-l"/home/chel/ti/TivaWare_C_Series-2.1.4.178/driverlib/ccs/Debug/driverlib.lib" \
-l"/home/chel/Documents/ARM/Util/Debug/Util.lib" \

-include ../makefile.init

RM := rm -rf
RMDIR := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include LCD/subdir_vars.mk
-include Receiver/subdir_vars.mk
-include Servo/subdir_vars.mk
-include Transmitter/subdir_vars.mk
-include subdir_rules.mk
-include LCD/subdir_rules.mk
-include Receiver/subdir_rules.mk
-include Servo/subdir_rules.mk
-include Transmitter/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
ECE397.out \

EXE_OUTPUTS__QUOTED += \
"ECE397.out" \

BIN_OUTPUTS += \
ECE397.hex \

BIN_OUTPUTS__QUOTED += \
"ECE397.hex" \


# All Target
all: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "ECE397.out"

# Tool invocations
ECE397.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: ARM Linker'
	"/home/chel/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.6.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --define=ccs="ccs" --define=PART_TM4C123GH6PM -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi -z -m"ECE397.map" --heap_size=1024 --stack_size=2048 -i"/home/chel/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.6.LTS/lib" -i"/home/chel/Documents/ARM/ECE397/Servo" -i"/home/chel/ti/TivaWare_C_Series-2.1.4.178" -i"/home/chel/Documents/ARM/Util" -i"/home/chel/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.6.LTS/include" --reread_libs --diag_wrap=off --display_error_number --warn_sections --xml_link_info="ECE397_linkInfo.xml" --rom_model -o "ECE397.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

ECE397.hex: $(EXE_OUTPUTS)
	@echo 'Building files: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo 'Invoking: ARM Hex Utility'
	"/home/chel/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.6.LTS/bin/armhex"  -o "ECE397.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(BIN_OUTPUTS__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "tm4c123gh6pm_startup_ccs.obj" "LCD/lcd.obj" "Receiver/receiver.obj" "Servo/servo.obj" "Servo/utilFunc.obj" "Transmitter/main.obj" "Transmitter/transmitter.obj" 
	-$(RM) "tm4c123gh6pm_startup_ccs.d" "LCD/lcd.d" "Receiver/receiver.d" "Servo/servo.d" "Servo/utilFunc.d" "Transmitter/main.d" "Transmitter/transmitter.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

