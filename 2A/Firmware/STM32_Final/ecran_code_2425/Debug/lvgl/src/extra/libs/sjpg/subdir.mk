################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lvgl/src/extra/libs/sjpg/lv_sjpg.c \
../lvgl/src/extra/libs/sjpg/tjpgd.c 

OBJS += \
./lvgl/src/extra/libs/sjpg/lv_sjpg.o \
./lvgl/src/extra/libs/sjpg/tjpgd.o 

C_DEPS += \
./lvgl/src/extra/libs/sjpg/lv_sjpg.d \
./lvgl/src/extra/libs/sjpg/tjpgd.d 


# Each subdirectory must supply rules for building sources it contributes
lvgl/src/extra/libs/sjpg/%.o lvgl/src/extra/libs/sjpg/%.su lvgl/src/extra/libs/sjpg/%.cyclo: ../lvgl/src/extra/libs/sjpg/%.c lvgl/src/extra/libs/sjpg/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32 -DSTM32F7 -DSTM32F746NGHx -DSTM32F746G_DISCO -DDEBUG -DSTM32F746xx -DUSE_HAL_DRIVER -c -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425" -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425/CMSIS/core" -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425/HAL_Driver/Inc" -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425/Utilities/STM32746G-Discovery" -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425/CMSIS/device" -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425/inc" -O3 -ffunction-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-lvgl-2f-src-2f-extra-2f-libs-2f-sjpg

clean-lvgl-2f-src-2f-extra-2f-libs-2f-sjpg:
	-$(RM) ./lvgl/src/extra/libs/sjpg/lv_sjpg.cyclo ./lvgl/src/extra/libs/sjpg/lv_sjpg.d ./lvgl/src/extra/libs/sjpg/lv_sjpg.o ./lvgl/src/extra/libs/sjpg/lv_sjpg.su ./lvgl/src/extra/libs/sjpg/tjpgd.cyclo ./lvgl/src/extra/libs/sjpg/tjpgd.d ./lvgl/src/extra/libs/sjpg/tjpgd.o ./lvgl/src/extra/libs/sjpg/tjpgd.su

.PHONY: clean-lvgl-2f-src-2f-extra-2f-libs-2f-sjpg

