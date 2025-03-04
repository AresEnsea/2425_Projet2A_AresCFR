################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lvgl/src/extra/others/monkey/lv_monkey.c 

OBJS += \
./lvgl/src/extra/others/monkey/lv_monkey.o 

C_DEPS += \
./lvgl/src/extra/others/monkey/lv_monkey.d 


# Each subdirectory must supply rules for building sources it contributes
lvgl/src/extra/others/monkey/%.o lvgl/src/extra/others/monkey/%.su lvgl/src/extra/others/monkey/%.cyclo: ../lvgl/src/extra/others/monkey/%.c lvgl/src/extra/others/monkey/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32 -DSTM32F7 -DSTM32F746NGHx -DSTM32F746G_DISCO -DDEBUG -DSTM32F746xx -DUSE_HAL_DRIVER -c -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425" -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425/CMSIS/core" -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425/HAL_Driver/Inc" -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425/Utilities/STM32746G-Discovery" -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425/CMSIS/device" -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425/inc" -O3 -ffunction-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-lvgl-2f-src-2f-extra-2f-others-2f-monkey

clean-lvgl-2f-src-2f-extra-2f-others-2f-monkey:
	-$(RM) ./lvgl/src/extra/others/monkey/lv_monkey.cyclo ./lvgl/src/extra/others/monkey/lv_monkey.d ./lvgl/src/extra/others/monkey/lv_monkey.o ./lvgl/src/extra/others/monkey/lv_monkey.su

.PHONY: clean-lvgl-2f-src-2f-extra-2f-others-2f-monkey

