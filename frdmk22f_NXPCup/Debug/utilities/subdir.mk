################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../utilities/fsl_assert.c \
../utilities/fsl_debug_console.c \
../utilities/fsl_str.c 

S_UPPER_SRCS += \
../utilities/fsl_memcpy.S 

C_DEPS += \
./utilities/fsl_assert.d \
./utilities/fsl_debug_console.d \
./utilities/fsl_str.d 

OBJS += \
./utilities/fsl_assert.o \
./utilities/fsl_debug_console.o \
./utilities/fsl_memcpy.o \
./utilities/fsl_str.o 


# Each subdirectory must supply rules for building sources it contributes
utilities/%.o: ../utilities/%.c utilities/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -D__REDLIB__ -DCPU_MK22FN512VLH12 -DCPU_MK22FN512VLH12_cm4 -DFRDM_K22F -DFREEDOM -DMCUXPRESSO_SDK -DSDK_DEBUGCONSOLE=1 -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=1 -DPRINTF_ADVANCED_ENABLE=1 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -DSDK_OS_BAREMETAL -I"C:\Users\guill\Documents\MCUXpressoIDE_11.9.0_2144\workspace\frdmk22f_ftm_simple_pwm_PWM\utilities" -I"C:\Users\guill\Documents\MCUXpressoIDE_11.9.0_2144\workspace\frdmk22f_ftm_simple_pwm_PWM\drivers" -I"C:\Users\guill\Documents\MCUXpressoIDE_11.9.0_2144\workspace\frdmk22f_ftm_simple_pwm_PWM\device" -I"C:\Users\guill\Documents\MCUXpressoIDE_11.9.0_2144\workspace\frdmk22f_ftm_simple_pwm_PWM\component\uart" -I"C:\Users\guill\Documents\MCUXpressoIDE_11.9.0_2144\workspace\frdmk22f_ftm_simple_pwm_PWM\component\lists" -I"C:\Users\guill\Documents\MCUXpressoIDE_11.9.0_2144\workspace\frdmk22f_ftm_simple_pwm_PWM\CMSIS" -I"C:\Users\guill\Documents\MCUXpressoIDE_11.9.0_2144\workspace\frdmk22f_ftm_simple_pwm_PWM\component\timer" -I"C:\Users\guill\Documents\MCUXpressoIDE_11.9.0_2144\workspace\frdmk22f_ftm_simple_pwm_PWM\source" -I"C:\Users\guill\Documents\MCUXpressoIDE_11.9.0_2144\workspace\frdmk22f_ftm_simple_pwm_PWM\board" -I"C:\Users\guill\Documents\MCUXpressoIDE_11.9.0_2144\workspace\frdmk22f_ftm_simple_pwm_PWM\frdmk22f\driver_examples\ftm\simple_pwm" -O0 -fno-common -g3 -gdwarf-4 -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

utilities/%.o: ../utilities/%.S utilities/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU Assembler'
	arm-none-eabi-gcc -c -x assembler-with-cpp -D__REDLIB__ -I"C:\Users\guill\Documents\MCUXpressoIDE_11.9.0_2144\workspace\frdmk22f_ftm_simple_pwm_PWM\utilities" -I"C:\Users\guill\Documents\MCUXpressoIDE_11.9.0_2144\workspace\frdmk22f_ftm_simple_pwm_PWM\drivers" -I"C:\Users\guill\Documents\MCUXpressoIDE_11.9.0_2144\workspace\frdmk22f_ftm_simple_pwm_PWM\device" -I"C:\Users\guill\Documents\MCUXpressoIDE_11.9.0_2144\workspace\frdmk22f_ftm_simple_pwm_PWM\component\uart" -I"C:\Users\guill\Documents\MCUXpressoIDE_11.9.0_2144\workspace\frdmk22f_ftm_simple_pwm_PWM\component\lists" -I"C:\Users\guill\Documents\MCUXpressoIDE_11.9.0_2144\workspace\frdmk22f_ftm_simple_pwm_PWM\CMSIS" -I"C:\Users\guill\Documents\MCUXpressoIDE_11.9.0_2144\workspace\frdmk22f_ftm_simple_pwm_PWM\component\timer" -I"C:\Users\guill\Documents\MCUXpressoIDE_11.9.0_2144\workspace\frdmk22f_ftm_simple_pwm_PWM\source" -I"C:\Users\guill\Documents\MCUXpressoIDE_11.9.0_2144\workspace\frdmk22f_ftm_simple_pwm_PWM\board" -I"C:\Users\guill\Documents\MCUXpressoIDE_11.9.0_2144\workspace\frdmk22f_ftm_simple_pwm_PWM\frdmk22f\driver_examples\ftm\simple_pwm" -g3 -gdwarf-4 -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -specs=redlib.specs -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-utilities

clean-utilities:
	-$(RM) ./utilities/fsl_assert.d ./utilities/fsl_assert.o ./utilities/fsl_debug_console.d ./utilities/fsl_debug_console.o ./utilities/fsl_memcpy.o ./utilities/fsl_str.d ./utilities/fsl_str.o

.PHONY: clean-utilities

