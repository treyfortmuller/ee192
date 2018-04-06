################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../source/telemetry/telemetry.c \
../source/telemetry/telemetry_uart.c 

OBJS += \
./source/telemetry/telemetry.o \
./source/telemetry/telemetry_uart.o 

C_DEPS += \
./source/telemetry/telemetry.d \
./source/telemetry/telemetry_uart.d 


# Each subdirectory must supply rules for building sources it contributes
source/telemetry/%.o: ../source/telemetry/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DPRINTF_FLOAT_ENABLE=0 -DFRDM_K64F -DCR_INTEGER_PRINTF -DFREEDOM -DSDK_DEBUGCONSOLE=0 -D__MCUXPRESSO -D__USE_CMSIS -DNDEBUG -DCPU_MK64FN1M0VLL12 -DCPU_MK64FN1M0VLL12_cm4 -I"C:\Users\Charlene Shong\Documents\MCUXpressoIDE_10.1.0_589\workspace\frdmk64f_telemetry\board" -I"C:\Users\Charlene Shong\Documents\MCUXpressoIDE_10.1.0_589\workspace\frdmk64f_telemetry\source" -I"C:\Users\Charlene Shong\Documents\MCUXpressoIDE_10.1.0_589\workspace\frdmk64f_telemetry" -I"C:\Users\Charlene Shong\Documents\MCUXpressoIDE_10.1.0_589\workspace\frdmk64f_telemetry\drivers" -I"C:\Users\Charlene Shong\Documents\MCUXpressoIDE_10.1.0_589\workspace\frdmk64f_telemetry\CMSIS" -I"C:\Users\Charlene Shong\Documents\MCUXpressoIDE_10.1.0_589\workspace\frdmk64f_telemetry\utilities" -I"C:\Users\Charlene Shong\Documents\MCUXpressoIDE_10.1.0_589\workspace\frdmk64f_telemetry\startup" -Os -fno-common -g -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


