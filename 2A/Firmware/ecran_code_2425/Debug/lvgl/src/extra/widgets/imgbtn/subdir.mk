################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lvgl/src/extra/widgets/imgbtn/lv_imgbtn.c 

OBJS += \
./lvgl/src/extra/widgets/imgbtn/lv_imgbtn.o 

C_DEPS += \
./lvgl/src/extra/widgets/imgbtn/lv_imgbtn.d 


# Each subdirectory must supply rules for building sources it contributes
lvgl/src/extra/widgets/imgbtn/%.o lvgl/src/extra/widgets/imgbtn/%.su lvgl/src/extra/widgets/imgbtn/%.cyclo: ../lvgl/src/extra/widgets/imgbtn/%.c lvgl/src/extra/widgets/imgbtn/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32 -DSTM32F7 -DSTM32F746NGHx -DSTM32F746G_DISCO -DDEBUG -DSTM32F746xx -DUSE_HAL_DRIVER -c -I"C:/Users/Antle/Downloads/2324_Projet2A_AresCFR-main (1)/2324_Projet2A_AresCFR-main/2A/Informatique/Main_Robot/ecran_code_2324_v2" -I"C:/Users/Antle/Downloads/2324_Projet2A_AresCFR-main (1)/2324_Projet2A_AresCFR-main/2A/Informatique/Main_Robot/ecran_code_2324_v2/CMSIS/core" -I"C:/Users/Antle/Downloads/2324_Projet2A_AresCFR-main (1)/2324_Projet2A_AresCFR-main/2A/Informatique/Main_Robot/ecran_code_2324_v2/HAL_Driver/Inc" -I"C:/Users/Antle/Downloads/2324_Projet2A_AresCFR-main (1)/2324_Projet2A_AresCFR-main/2A/Informatique/Main_Robot/ecran_code_2324_v2/Utilities/STM32746G-Discovery" -I"C:/Users/Antle/Downloads/2324_Projet2A_AresCFR-main (1)/2324_Projet2A_AresCFR-main/2A/Informatique/Main_Robot/ecran_code_2324_v2/CMSIS/device" -I"C:/Users/Antle/Downloads/2324_Projet2A_AresCFR-main (1)/2324_Projet2A_AresCFR-main/2A/Informatique/Main_Robot/ecran_code_2324_v2/inc" -O3 -ffunction-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-lvgl-2f-src-2f-extra-2f-widgets-2f-imgbtn

clean-lvgl-2f-src-2f-extra-2f-widgets-2f-imgbtn:
	-$(RM) ./lvgl/src/extra/widgets/imgbtn/lv_imgbtn.cyclo ./lvgl/src/extra/widgets/imgbtn/lv_imgbtn.d ./lvgl/src/extra/widgets/imgbtn/lv_imgbtn.o ./lvgl/src/extra/widgets/imgbtn/lv_imgbtn.su

.PHONY: clean-lvgl-2f-src-2f-extra-2f-widgets-2f-imgbtn

