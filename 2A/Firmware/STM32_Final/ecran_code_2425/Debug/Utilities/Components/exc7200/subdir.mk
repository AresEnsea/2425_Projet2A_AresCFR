################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Utilities/Components/exc7200/exc7200.c 

OBJS += \
./Utilities/Components/exc7200/exc7200.o 

C_DEPS += \
./Utilities/Components/exc7200/exc7200.d 


# Each subdirectory must supply rules for building sources it contributes
Utilities/Components/exc7200/%.o Utilities/Components/exc7200/%.su Utilities/Components/exc7200/%.cyclo: ../Utilities/Components/exc7200/%.c Utilities/Components/exc7200/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32 -DSTM32F7 -DSTM32F746NGHx -DSTM32F746G_DISCO -DDEBUG -DSTM32F746xx -DUSE_HAL_DRIVER -c -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425" -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425/CMSIS/core" -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425/HAL_Driver/Inc" -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425/Utilities/STM32746G-Discovery" -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425/CMSIS/device" -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425/inc" -O3 -ffunction-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Utilities-2f-Components-2f-exc7200

clean-Utilities-2f-Components-2f-exc7200:
	-$(RM) ./Utilities/Components/exc7200/exc7200.cyclo ./Utilities/Components/exc7200/exc7200.d ./Utilities/Components/exc7200/exc7200.o ./Utilities/Components/exc7200/exc7200.su

.PHONY: clean-Utilities-2f-Components-2f-exc7200

