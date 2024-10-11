################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Chay_di_lam_on/Core/Startup/startup_stm32f401ccux.s 

OBJS += \
./Chay_di_lam_on/Core/Startup/startup_stm32f401ccux.o 

S_DEPS += \
./Chay_di_lam_on/Core/Startup/startup_stm32f401ccux.d 


# Each subdirectory must supply rules for building sources it contributes
Chay_di_lam_on/Core/Startup/%.o: ../Chay_di_lam_on/Core/Startup/%.s Chay_di_lam_on/Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Chay_di_lam_on-2f-Core-2f-Startup

clean-Chay_di_lam_on-2f-Core-2f-Startup:
	-$(RM) ./Chay_di_lam_on/Core/Startup/startup_stm32f401ccux.d ./Chay_di_lam_on/Core/Startup/startup_stm32f401ccux.o

.PHONY: clean-Chay_di_lam_on-2f-Core-2f-Startup

