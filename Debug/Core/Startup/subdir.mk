################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32l151c8txa.s 

OBJS += \
./Core/Startup/startup_stm32l151c8txa.o 

S_DEPS += \
./Core/Startup/startup_stm32l151c8txa.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/startup_stm32l151c8txa.o: ../Core/Startup/startup_stm32l151c8txa.s
	arm-none-eabi-gcc -mcpu=cortex-m3 -g3 -c -x assembler-with-cpp -MMD -MP -MF"Core/Startup/startup_stm32l151c8txa.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

