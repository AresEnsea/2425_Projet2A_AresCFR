################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Utilities/STM32746G-Discovery/stm32746g_discovery.c \
../Utilities/STM32746G-Discovery/stm32746g_discovery_audio.c \
../Utilities/STM32746G-Discovery/stm32746g_discovery_camera.c \
../Utilities/STM32746G-Discovery/stm32746g_discovery_eeprom.c \
../Utilities/STM32746G-Discovery/stm32746g_discovery_qspi.c \
../Utilities/STM32746G-Discovery/stm32746g_discovery_sd.c \
../Utilities/STM32746G-Discovery/stm32746g_discovery_sdram.c \
../Utilities/STM32746G-Discovery/stm32746g_discovery_ts.c 

OBJS += \
./Utilities/STM32746G-Discovery/stm32746g_discovery.o \
./Utilities/STM32746G-Discovery/stm32746g_discovery_audio.o \
./Utilities/STM32746G-Discovery/stm32746g_discovery_camera.o \
./Utilities/STM32746G-Discovery/stm32746g_discovery_eeprom.o \
./Utilities/STM32746G-Discovery/stm32746g_discovery_qspi.o \
./Utilities/STM32746G-Discovery/stm32746g_discovery_sd.o \
./Utilities/STM32746G-Discovery/stm32746g_discovery_sdram.o \
./Utilities/STM32746G-Discovery/stm32746g_discovery_ts.o 

C_DEPS += \
./Utilities/STM32746G-Discovery/stm32746g_discovery.d \
./Utilities/STM32746G-Discovery/stm32746g_discovery_audio.d \
./Utilities/STM32746G-Discovery/stm32746g_discovery_camera.d \
./Utilities/STM32746G-Discovery/stm32746g_discovery_eeprom.d \
./Utilities/STM32746G-Discovery/stm32746g_discovery_qspi.d \
./Utilities/STM32746G-Discovery/stm32746g_discovery_sd.d \
./Utilities/STM32746G-Discovery/stm32746g_discovery_sdram.d \
./Utilities/STM32746G-Discovery/stm32746g_discovery_ts.d 


# Each subdirectory must supply rules for building sources it contributes
Utilities/STM32746G-Discovery/%.o Utilities/STM32746G-Discovery/%.su Utilities/STM32746G-Discovery/%.cyclo: ../Utilities/STM32746G-Discovery/%.c Utilities/STM32746G-Discovery/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32 -DSTM32F7 -DSTM32F746NGHx -DSTM32F746G_DISCO -DDEBUG -DSTM32F746xx -DUSE_HAL_DRIVER -c -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425" -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425/CMSIS/core" -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425/HAL_Driver/Inc" -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425/Utilities/STM32746G-Discovery" -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425/CMSIS/device" -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425/inc" -O3 -ffunction-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Utilities-2f-STM32746G-2d-Discovery

clean-Utilities-2f-STM32746G-2d-Discovery:
	-$(RM) ./Utilities/STM32746G-Discovery/stm32746g_discovery.cyclo ./Utilities/STM32746G-Discovery/stm32746g_discovery.d ./Utilities/STM32746G-Discovery/stm32746g_discovery.o ./Utilities/STM32746G-Discovery/stm32746g_discovery.su ./Utilities/STM32746G-Discovery/stm32746g_discovery_audio.cyclo ./Utilities/STM32746G-Discovery/stm32746g_discovery_audio.d ./Utilities/STM32746G-Discovery/stm32746g_discovery_audio.o ./Utilities/STM32746G-Discovery/stm32746g_discovery_audio.su ./Utilities/STM32746G-Discovery/stm32746g_discovery_camera.cyclo ./Utilities/STM32746G-Discovery/stm32746g_discovery_camera.d ./Utilities/STM32746G-Discovery/stm32746g_discovery_camera.o ./Utilities/STM32746G-Discovery/stm32746g_discovery_camera.su ./Utilities/STM32746G-Discovery/stm32746g_discovery_eeprom.cyclo ./Utilities/STM32746G-Discovery/stm32746g_discovery_eeprom.d ./Utilities/STM32746G-Discovery/stm32746g_discovery_eeprom.o ./Utilities/STM32746G-Discovery/stm32746g_discovery_eeprom.su ./Utilities/STM32746G-Discovery/stm32746g_discovery_qspi.cyclo ./Utilities/STM32746G-Discovery/stm32746g_discovery_qspi.d ./Utilities/STM32746G-Discovery/stm32746g_discovery_qspi.o ./Utilities/STM32746G-Discovery/stm32746g_discovery_qspi.su ./Utilities/STM32746G-Discovery/stm32746g_discovery_sd.cyclo ./Utilities/STM32746G-Discovery/stm32746g_discovery_sd.d ./Utilities/STM32746G-Discovery/stm32746g_discovery_sd.o ./Utilities/STM32746G-Discovery/stm32746g_discovery_sd.su ./Utilities/STM32746G-Discovery/stm32746g_discovery_sdram.cyclo ./Utilities/STM32746G-Discovery/stm32746g_discovery_sdram.d ./Utilities/STM32746G-Discovery/stm32746g_discovery_sdram.o ./Utilities/STM32746G-Discovery/stm32746g_discovery_sdram.su ./Utilities/STM32746G-Discovery/stm32746g_discovery_ts.cyclo ./Utilities/STM32746G-Discovery/stm32746g_discovery_ts.d ./Utilities/STM32746G-Discovery/stm32746g_discovery_ts.o ./Utilities/STM32746G-Discovery/stm32746g_discovery_ts.su

.PHONY: clean-Utilities-2f-STM32746G-2d-Discovery

