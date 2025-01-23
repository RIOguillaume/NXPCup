################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../component/timer_manager/fsl_component_timer_manager.c 

C_DEPS += \
./component/timer_manager/fsl_component_timer_manager.d 

OBJS += \
./component/timer_manager/fsl_component_timer_manager.o 


# Each subdirectory must supply rules for building sources it contributes
component/timer_manager/%.o: ../component/timer_manager/%.c component/timer_manager/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -D__REDLIB__ -DCPU_MK22FN512VLH12 -DCPU_MK22FN512VLH12_cm4 -DFRDM_K22F -DFREEDOM -DMCUXPRESSO_SDK -DSDK_DEBUGCONSOLE=1 -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=1 -DPRINTF_ADVANCED_ENABLE=1 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -DSDK_OS_BAREMETAL -I"C:\Users\guill\Documents\MCUXpressoIDE_11.9.0_2144\workspace\frdmk22f_ftm_simple_pwm_PWM\utilities" -I"C:\Users\guill\Documents\MCUXpressoIDE_11.9.0_2144\workspace\frdmk22f_ftm_simple_pwm_PWM\drivers" -I"C:\Users\guill\Documents\MCUXpressoIDE_11.9.0_2144\workspace\frdmk22f_ftm_simple_pwm_PWM\device" -I"C:\Users\guill\Documents\MCUXpressoIDE_11.9.0_2144\workspace\frdmk22f_ftm_simple_pwm_PWM\component\uart" -I"C:\Users\guill\Documents\MCUXpressoIDE_11.9.0_2144\workspace\frdmk22f_ftm_simple_pwm_PWM\component\lists" -I"C:\Users\guill\Documents\MCUXpressoIDE_11.9.0_2144\workspace\frdmk22f_ftm_simple_pwm_PWM\CMSIS" -I"C:\Users\guill\Documents\MCUXpressoIDE_11.9.0_2144\workspace\frdmk22f_ftm_simple_pwm_PWM\component\timer_manager" -I"C:\Users\guill\Documents\MCUXpressoIDE_11.9.0_2144\workspace\frdmk22f_ftm_simple_pwm_PWM\component\timer" -I"C:\Users\guill\Documents\MCUXpressoIDE_11.9.0_2144\workspace\frdmk22f_ftm_simple_pwm_PWM\source" -I"C:\Users\guill\Documents\MCUXpressoIDE_11.9.0_2144\workspace\frdmk22f_ftm_simple_pwm_PWM\board" -I"C:\Users\guill\Documents\MCUXpressoIDE_11.9.0_2144\workspace\frdmk22f_ftm_simple_pwm_PWM\frdmk22f\driver_examples\ftm\simple_pwm" -O0 -fno-common -g3 -gdwarf-4 -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-component-2f-timer_manager

clean-component-2f-timer_manager:
	-$(RM) ./component/timer_manager/fsl_component_timer_manager.d ./component/timer_manager/fsl_component_timer_manager.o

.PHONY: clean-component-2f-timer_manager

