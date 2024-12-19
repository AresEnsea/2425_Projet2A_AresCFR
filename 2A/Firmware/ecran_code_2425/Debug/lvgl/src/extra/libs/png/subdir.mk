################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lvgl/src/extra/libs/png/lodepng.c \
../lvgl/src/extra/libs/png/lv_png.c 

OBJS += \
./lvgl/src/extra/libs/png/lodepng.o \
./lvgl/src/extra/libs/png/lv_png.o 

C_DEPS += \
./lvgl/src/extra/libs/png/lodepng.d \
./lvgl/src/extra/libs/png/lv_png.d 


# Each subdirectory must supply rules for building sources it contributes
lvgl/src/extra/libs/png/%.o lvgl/src/extra/libs/png/%.su lvgl/src/extra/libs/png/%.cyclo: ../lvgl/src/extra/libs/png/%.c lvgl/src/extra/libs/png/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32 -DSTM32F7 -DSTM32F746NGHx -DSTM32F746G_DISCO -DDEBUG -DSTM32F746xx -DUSE_HAL_DRIVER -c -I"C:/Users/Antle/Downloads/2324_Projet2A_AresCFR-main (1)/2324_Projet2A_AresCFR-main/2A/Informatique/Main_Robot/ecran_code_2324_v2" -I"C:/Users/Antle/Downloads/2324_Projet2A_AresCFR-main (1)/2324_Projet2A_AresCFR-main/2A/Informatique/Main_Robot/ecran_code_2324_v2/CMSIS/core" -I"C:/Users/Antle/Downloads/2324_Projet2A_AresCFR-main (1)/2324_Projet2A_AresCFR-main/2A/Informatique/Main_Robot/ecran_code_2324_v2/HAL_Driver/Inc" -I"C:/Users/Antle/Downloads/2324_Projet2A_AresCFR-main (1)/2324_Projet2A_AresCFR-main/2A/Informatique/Main_Robot/ecran_code_2324_v2/Utilities/STM32746G-Discovery" -I"C:/Users/Antle/Downloads/2324_Projet2A_AresCFR-main (1)/2324_Projet2A_AresCFR-main/2A/Informatique/Main_Robot/ecran_code_2324_v2/CMSIS/device" -I"C:/Users/Antle/Downloads/2324_Projet2A_AresCFR-main (1)/2324_Projet2A_AresCFR-main/2A/Informatique/Main_Robot/ecran_code_2324_v2/inc" -O3 -ffunction-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-lvgl-2f-src-2f-extra-2f-libs-2f-png

clean-lvgl-2f-src-2f-extra-2f-libs-2f-png:
	-$(RM) ./lvgl/src/extra/libs/png/lodepng.cyclo ./lvgl/src/extra/libs/png/lodepng.d ./lvgl/src/extra/libs/png/lodepng.o ./lvgl/src/extra/libs/png/lodepng.su ./lvgl/src/extra/libs/png/lv_png.cyclo ./lvgl/src/extra/libs/png/lv_png.d ./lvgl/src/extra/libs/png/lv_png.o ./lvgl/src/extra/libs/png/lv_png.su

.PHONY: clean-lvgl-2f-src-2f-extra-2f-libs-2f-png

