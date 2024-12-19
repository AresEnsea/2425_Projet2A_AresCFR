################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/main.c \
../src/score.c \
../src/stm32f7xx_it.c \
../src/strategie.c \
../src/syscalls.c \
../src/system_stm32f7xx.c 

OBJS += \
./src/main.o \
./src/score.o \
./src/stm32f7xx_it.o \
./src/strategie.o \
./src/syscalls.o \
./src/system_stm32f7xx.o 

C_DEPS += \
./src/main.d \
./src/score.d \
./src/stm32f7xx_it.d \
./src/strategie.d \
./src/syscalls.d \
./src/system_stm32f7xx.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o src/%.su src/%.cyclo: ../src/%.c src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32 -DSTM32F7 -DSTM32F746NGHx -DSTM32F746G_DISCO -DDEBUG -DSTM32F746xx -DUSE_HAL_DRIVER -c -I"C:/Users/Antle/Downloads/2324_Projet2A_AresCFR-main (1)/2324_Projet2A_AresCFR-main/2A/Informatique/Main_Robot/ecran_code_2324_v2" -I"C:/Users/Antle/Downloads/2324_Projet2A_AresCFR-main (1)/2324_Projet2A_AresCFR-main/2A/Informatique/Main_Robot/ecran_code_2324_v2/CMSIS/core" -I"C:/Users/Antle/Downloads/2324_Projet2A_AresCFR-main (1)/2324_Projet2A_AresCFR-main/2A/Informatique/Main_Robot/ecran_code_2324_v2/HAL_Driver/Inc" -I"C:/Users/Antle/Downloads/2324_Projet2A_AresCFR-main (1)/2324_Projet2A_AresCFR-main/2A/Informatique/Main_Robot/ecran_code_2324_v2/Utilities/STM32746G-Discovery" -I"C:/Users/Antle/Downloads/2324_Projet2A_AresCFR-main (1)/2324_Projet2A_AresCFR-main/2A/Informatique/Main_Robot/ecran_code_2324_v2/CMSIS/device" -I"C:/Users/Antle/Downloads/2324_Projet2A_AresCFR-main (1)/2324_Projet2A_AresCFR-main/2A/Informatique/Main_Robot/ecran_code_2324_v2/inc" -O3 -ffunction-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-src

clean-src:
	-$(RM) ./src/main.cyclo ./src/main.d ./src/main.o ./src/main.su ./src/score.cyclo ./src/score.d ./src/score.o ./src/score.su ./src/stm32f7xx_it.cyclo ./src/stm32f7xx_it.d ./src/stm32f7xx_it.o ./src/stm32f7xx_it.su ./src/strategie.cyclo ./src/strategie.d ./src/strategie.o ./src/strategie.su ./src/syscalls.cyclo ./src/syscalls.d ./src/syscalls.o ./src/syscalls.su ./src/system_stm32f7xx.cyclo ./src/system_stm32f7xx.d ./src/system_stm32f7xx.o ./src/system_stm32f7xx.su

.PHONY: clean-src

