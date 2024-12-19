################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../HAL_Driver/Src/Legacy/stm32f7xx_hal_can.c 

OBJS += \
./HAL_Driver/Src/Legacy/stm32f7xx_hal_can.o 

C_DEPS += \
./HAL_Driver/Src/Legacy/stm32f7xx_hal_can.d 


# Each subdirectory must supply rules for building sources it contributes
HAL_Driver/Src/Legacy/%.o HAL_Driver/Src/Legacy/%.su HAL_Driver/Src/Legacy/%.cyclo: ../HAL_Driver/Src/Legacy/%.c HAL_Driver/Src/Legacy/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32 -DSTM32F7 -DSTM32F746NGHx -DSTM32F746G_DISCO -DDEBUG -DSTM32F746xx -DUSE_HAL_DRIVER -c -I"C:/Users/Antle/Downloads/2324_Projet2A_AresCFR-main (1)/2324_Projet2A_AresCFR-main/2A/Informatique/Main_Robot/ecran_code_2324_v2" -I"C:/Users/Antle/Downloads/2324_Projet2A_AresCFR-main (1)/2324_Projet2A_AresCFR-main/2A/Informatique/Main_Robot/ecran_code_2324_v2/CMSIS/core" -I"C:/Users/Antle/Downloads/2324_Projet2A_AresCFR-main (1)/2324_Projet2A_AresCFR-main/2A/Informatique/Main_Robot/ecran_code_2324_v2/HAL_Driver/Inc" -I"C:/Users/Antle/Downloads/2324_Projet2A_AresCFR-main (1)/2324_Projet2A_AresCFR-main/2A/Informatique/Main_Robot/ecran_code_2324_v2/Utilities/STM32746G-Discovery" -I"C:/Users/Antle/Downloads/2324_Projet2A_AresCFR-main (1)/2324_Projet2A_AresCFR-main/2A/Informatique/Main_Robot/ecran_code_2324_v2/CMSIS/device" -I"C:/Users/Antle/Downloads/2324_Projet2A_AresCFR-main (1)/2324_Projet2A_AresCFR-main/2A/Informatique/Main_Robot/ecran_code_2324_v2/inc" -O3 -ffunction-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-HAL_Driver-2f-Src-2f-Legacy

clean-HAL_Driver-2f-Src-2f-Legacy:
	-$(RM) ./HAL_Driver/Src/Legacy/stm32f7xx_hal_can.cyclo ./HAL_Driver/Src/Legacy/stm32f7xx_hal_can.d ./HAL_Driver/Src/Legacy/stm32f7xx_hal_can.o ./HAL_Driver/Src/Legacy/stm32f7xx_hal_can.su

.PHONY: clean-HAL_Driver-2f-Src-2f-Legacy

