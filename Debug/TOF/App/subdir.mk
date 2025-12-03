################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../TOF/App/app_tof.c 

OBJS += \
./TOF/App/app_tof.o 

C_DEPS += \
./TOF/App/app_tof.d 


# Each subdirectory must supply rules for building sources it contributes
TOF/App/%.o TOF/App/%.su TOF/App/%.cyclo: ../TOF/App/%.c TOF/App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../TOF/App -I../TOF/Target -I../Drivers/BSP/STM32F4xx_Nucleo -I../Drivers/BSP/Components/vl53l8cx/modules -I../Drivers/BSP/Components/vl53l8cx/porting -I../Drivers/BSP/Components/vl53l8cx -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-TOF-2f-App

clean-TOF-2f-App:
	-$(RM) ./TOF/App/app_tof.cyclo ./TOF/App/app_tof.d ./TOF/App/app_tof.o ./TOF/App/app_tof.su

.PHONY: clean-TOF-2f-App

