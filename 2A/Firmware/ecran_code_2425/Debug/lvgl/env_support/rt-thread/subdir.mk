################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lvgl/env_support/rt-thread/lv_rt_thread_port.c 

OBJS += \
./lvgl/env_support/rt-thread/lv_rt_thread_port.o 

C_DEPS += \
./lvgl/env_support/rt-thread/lv_rt_thread_port.d 


# Each subdirectory must supply rules for building sources it contributes
lvgl/env_support/rt-thread/%.o lvgl/env_support/rt-thread/%.su lvgl/env_support/rt-thread/%.cyclo: ../lvgl/env_support/rt-thread/%.c lvgl/env_support/rt-thread/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32 -DSTM32F7 -DSTM32F746NGHx -DSTM32F746G_DISCO -DDEBUG -DSTM32F746xx -DUSE_HAL_DRIVER -c -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425" -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425/CMSIS/core" -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425/HAL_Driver/Inc" -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425/Utilities/STM32746G-Discovery" -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425/CMSIS/device" -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425/inc" -O3 -ffunction-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-lvgl-2f-env_support-2f-rt-2d-thread

clean-lvgl-2f-env_support-2f-rt-2d-thread:
	-$(RM) ./lvgl/env_support/rt-thread/lv_rt_thread_port.cyclo ./lvgl/env_support/rt-thread/lv_rt_thread_port.d ./lvgl/env_support/rt-thread/lv_rt_thread_port.o ./lvgl/env_support/rt-thread/lv_rt_thread_port.su

.PHONY: clean-lvgl-2f-env_support-2f-rt-2d-thread

