################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/DefaultFonts.c \
../Src/I2C.c \
../Src/ST7735.c \
../Src/hw_config.c \
../Src/main.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/system_stm32f4xx.c 

OBJS += \
./Src/DefaultFonts.o \
./Src/I2C.o \
./Src/ST7735.o \
./Src/hw_config.o \
./Src/main.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/system_stm32f4xx.o 

C_DEPS += \
./Src/DefaultFonts.d \
./Src/I2C.d \
./Src/ST7735.d \
./Src/hw_config.d \
./Src/main.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F401RETx -DDEBUG -c -I"D:/GITHUB/STM32/EDS_2023/CMSIS/Include" -I"D:/GITHUB/STM32/EDS_2023/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/GITHUB/STM32/EDS_2023/NUCLEO_ST7735_I2C/Inc" -I"D:/GITHUB/STM32/EDS_2023/MCAL_F40x/Inc" -I"D:/GITHUB/STM32/EDS_2023/MCAL_F40x/Inc/mcalTimer" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/DefaultFonts.cyclo ./Src/DefaultFonts.d ./Src/DefaultFonts.o ./Src/DefaultFonts.su ./Src/I2C.cyclo ./Src/I2C.d ./Src/I2C.o ./Src/I2C.su ./Src/ST7735.cyclo ./Src/ST7735.d ./Src/ST7735.o ./Src/ST7735.su ./Src/hw_config.cyclo ./Src/hw_config.d ./Src/hw_config.o ./Src/hw_config.su ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/system_stm32f4xx.cyclo ./Src/system_stm32f4xx.d ./Src/system_stm32f4xx.o ./Src/system_stm32f4xx.su

.PHONY: clean-Src

