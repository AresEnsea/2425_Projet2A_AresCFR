################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Image/TableRedi.c \
../Image/n0.c \
../Image/n1.c \
../Image/n2.c \
../Image/n3.c \
../Image/n4.c \
../Image/n5.c \
../Image/n6.c \
../Image/n7.c \
../Image/n8.c \
../Image/n9.c 

OBJS += \
./Image/TableRedi.o \
./Image/n0.o \
./Image/n1.o \
./Image/n2.o \
./Image/n3.o \
./Image/n4.o \
./Image/n5.o \
./Image/n6.o \
./Image/n7.o \
./Image/n8.o \
./Image/n9.o 

C_DEPS += \
./Image/TableRedi.d \
./Image/n0.d \
./Image/n1.d \
./Image/n2.d \
./Image/n3.d \
./Image/n4.d \
./Image/n5.d \
./Image/n6.d \
./Image/n7.d \
./Image/n8.d \
./Image/n9.d 


# Each subdirectory must supply rules for building sources it contributes
Image/%.o Image/%.su Image/%.cyclo: ../Image/%.c Image/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32 -DSTM32F7 -DSTM32F746NGHx -DSTM32F746G_DISCO -DDEBUG -DSTM32F746xx -DUSE_HAL_DRIVER -c -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425" -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425/CMSIS/core" -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425/HAL_Driver/Inc" -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425/Utilities/STM32746G-Discovery" -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425/CMSIS/device" -I"C:/Users/Antle/STM32CubeIDE/workspace_1.16.1/ecran_code_2425/inc" -O3 -ffunction-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Image

clean-Image:
	-$(RM) ./Image/TableRedi.cyclo ./Image/TableRedi.d ./Image/TableRedi.o ./Image/TableRedi.su ./Image/n0.cyclo ./Image/n0.d ./Image/n0.o ./Image/n0.su ./Image/n1.cyclo ./Image/n1.d ./Image/n1.o ./Image/n1.su ./Image/n2.cyclo ./Image/n2.d ./Image/n2.o ./Image/n2.su ./Image/n3.cyclo ./Image/n3.d ./Image/n3.o ./Image/n3.su ./Image/n4.cyclo ./Image/n4.d ./Image/n4.o ./Image/n4.su ./Image/n5.cyclo ./Image/n5.d ./Image/n5.o ./Image/n5.su ./Image/n6.cyclo ./Image/n6.d ./Image/n6.o ./Image/n6.su ./Image/n7.cyclo ./Image/n7.d ./Image/n7.o ./Image/n7.su ./Image/n8.cyclo ./Image/n8.d ./Image/n8.o ./Image/n8.su ./Image/n9.cyclo ./Image/n9.d ./Image/n9.o ./Image/n9.su

.PHONY: clean-Image

