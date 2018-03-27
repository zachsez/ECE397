################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
tm4c123gh6pm_startup_ccs.obj: ../tm4c123gh6pm_startup_ccs.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"/home/chel/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.6.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="/home/chel/Documents/ARM/ECE397/Receiver" --include_path="/home/chel/Documents/ARM/ECE397" --include_path="/home/chel/Documents/ARM/ECE397/LCD" --include_path="/home/chel/Documents/ARM/ECE397/Servo" --include_path="/home/chel/ti/TivaWare_C_Series-2.1.4.178" --include_path="/home/chel/Documents/ARM/Util" --include_path="/home/chel/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.6.LTS/include" --define=ccs="ccs" --define=PART_TM4C123GH6PM -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="tm4c123gh6pm_startup_ccs.d_raw" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: "$<"'
	@echo ' '


