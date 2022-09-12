################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/stm32f429zi_gpio_driver.c \
../Drivers/Src/stm32f429zi_i2c_driver.c \
../Drivers/Src/stm32f429zi_rcc_driver.c \
../Drivers/Src/stm32f429zi_spi_driver.c \
../Drivers/Src/stm32f429zi_usart_driver.c 

OBJS += \
./Drivers/Src/stm32f429zi_gpio_driver.o \
./Drivers/Src/stm32f429zi_i2c_driver.o \
./Drivers/Src/stm32f429zi_rcc_driver.o \
./Drivers/Src/stm32f429zi_spi_driver.o \
./Drivers/Src/stm32f429zi_usart_driver.o 

C_DEPS += \
./Drivers/Src/stm32f429zi_gpio_driver.d \
./Drivers/Src/stm32f429zi_i2c_driver.d \
./Drivers/Src/stm32f429zi_rcc_driver.d \
./Drivers/Src/stm32f429zi_spi_driver.d \
./Drivers/Src/stm32f429zi_usart_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/%.o Drivers/Src/%.su: ../Drivers/Src/%.c Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F429I_DISC1 -DSTM32 -DSTM32F429ZITx -DSTM32F4 -c -I"C:/Users/pc/Documents/GitHub/Driver-Development-STM32F429ZI/stm32f429zi_drivers/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Src

clean-Drivers-2f-Src:
	-$(RM) ./Drivers/Src/stm32f429zi_gpio_driver.d ./Drivers/Src/stm32f429zi_gpio_driver.o ./Drivers/Src/stm32f429zi_gpio_driver.su ./Drivers/Src/stm32f429zi_i2c_driver.d ./Drivers/Src/stm32f429zi_i2c_driver.o ./Drivers/Src/stm32f429zi_i2c_driver.su ./Drivers/Src/stm32f429zi_rcc_driver.d ./Drivers/Src/stm32f429zi_rcc_driver.o ./Drivers/Src/stm32f429zi_rcc_driver.su ./Drivers/Src/stm32f429zi_spi_driver.d ./Drivers/Src/stm32f429zi_spi_driver.o ./Drivers/Src/stm32f429zi_spi_driver.su ./Drivers/Src/stm32f429zi_usart_driver.d ./Drivers/Src/stm32f429zi_usart_driver.o ./Drivers/Src/stm32f429zi_usart_driver.su

.PHONY: clean-Drivers-2f-Src

