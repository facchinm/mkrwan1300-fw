################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Utilities/low_power_manager.c \
/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Utilities/queue.c \
/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Utilities/systime.c \
/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Utilities/timeServer.c \
/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Utilities/trace.c \
/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Utilities/utilities.c 

OBJS += \
./Middlewares/LoRaWAN/Utilities/low_power_manager.o \
./Middlewares/LoRaWAN/Utilities/queue.o \
./Middlewares/LoRaWAN/Utilities/systime.o \
./Middlewares/LoRaWAN/Utilities/timeServer.o \
./Middlewares/LoRaWAN/Utilities/trace.o \
./Middlewares/LoRaWAN/Utilities/utilities.o 

C_DEPS += \
./Middlewares/LoRaWAN/Utilities/low_power_manager.d \
./Middlewares/LoRaWAN/Utilities/queue.d \
./Middlewares/LoRaWAN/Utilities/systime.d \
./Middlewares/LoRaWAN/Utilities/timeServer.d \
./Middlewares/LoRaWAN/Utilities/trace.d \
./Middlewares/LoRaWAN/Utilities/utilities.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/LoRaWAN/Utilities/low_power_manager.o: /run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Utilities/low_power_manager.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -DSTM32L072xx -DUSE_B_L072Z_LRWAN1 -DUSE_FULL_LL_DRIVER -DNO_MAC_PRINTF -DREGION_EU868 -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Projects/B-L072Z-LRWAN1/Applications/LoRa/AT_Slave/Core/inc" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Projects/B-L072Z-LRWAN1/Applications/LoRa/AT_Slave/LoRaWAN/App/inc" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Drivers/BSP/CMWX1ZZABZ-0xx" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Drivers/STM32L0xx_HAL_Driver/Inc" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Drivers/CMSIS/Device/ST/STM32L0xx/Include" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Drivers/CMSIS/Include" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Crypto" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Mac" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Phy" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Utilities" -I../../../../../../../../Drivers/BSP/X_NUCLEO_IKS01A1 -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Patterns/Basic" -I../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../../Drivers/BSP/Components/lps22hb -I../../../../../../../../Drivers/BSP/Components/lps25hb -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Drivers/BSP/Components/sx1276" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Drivers/BSP/B-L072Z-LRWAN1"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Middlewares/LoRaWAN/Utilities/queue.o: /run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Utilities/queue.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -DSTM32L072xx -DUSE_B_L072Z_LRWAN1 -DUSE_FULL_LL_DRIVER -DNO_MAC_PRINTF -DREGION_EU868 -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Projects/B-L072Z-LRWAN1/Applications/LoRa/AT_Slave/Core/inc" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Projects/B-L072Z-LRWAN1/Applications/LoRa/AT_Slave/LoRaWAN/App/inc" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Drivers/BSP/CMWX1ZZABZ-0xx" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Drivers/STM32L0xx_HAL_Driver/Inc" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Drivers/CMSIS/Device/ST/STM32L0xx/Include" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Drivers/CMSIS/Include" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Crypto" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Mac" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Phy" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Utilities" -I../../../../../../../../Drivers/BSP/X_NUCLEO_IKS01A1 -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Patterns/Basic" -I../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../../Drivers/BSP/Components/lps22hb -I../../../../../../../../Drivers/BSP/Components/lps25hb -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Drivers/BSP/Components/sx1276" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Drivers/BSP/B-L072Z-LRWAN1"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Middlewares/LoRaWAN/Utilities/systime.o: /run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Utilities/systime.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -DSTM32L072xx -DUSE_B_L072Z_LRWAN1 -DUSE_FULL_LL_DRIVER -DNO_MAC_PRINTF -DREGION_EU868 -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Projects/B-L072Z-LRWAN1/Applications/LoRa/AT_Slave/Core/inc" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Projects/B-L072Z-LRWAN1/Applications/LoRa/AT_Slave/LoRaWAN/App/inc" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Drivers/BSP/CMWX1ZZABZ-0xx" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Drivers/STM32L0xx_HAL_Driver/Inc" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Drivers/CMSIS/Device/ST/STM32L0xx/Include" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Drivers/CMSIS/Include" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Crypto" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Mac" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Phy" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Utilities" -I../../../../../../../../Drivers/BSP/X_NUCLEO_IKS01A1 -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Patterns/Basic" -I../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../../Drivers/BSP/Components/lps22hb -I../../../../../../../../Drivers/BSP/Components/lps25hb -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Drivers/BSP/Components/sx1276" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Drivers/BSP/B-L072Z-LRWAN1"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Middlewares/LoRaWAN/Utilities/timeServer.o: /run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Utilities/timeServer.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -DSTM32L072xx -DUSE_B_L072Z_LRWAN1 -DUSE_FULL_LL_DRIVER -DNO_MAC_PRINTF -DREGION_EU868 -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Projects/B-L072Z-LRWAN1/Applications/LoRa/AT_Slave/Core/inc" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Projects/B-L072Z-LRWAN1/Applications/LoRa/AT_Slave/LoRaWAN/App/inc" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Drivers/BSP/CMWX1ZZABZ-0xx" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Drivers/STM32L0xx_HAL_Driver/Inc" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Drivers/CMSIS/Device/ST/STM32L0xx/Include" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Drivers/CMSIS/Include" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Crypto" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Mac" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Phy" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Utilities" -I../../../../../../../../Drivers/BSP/X_NUCLEO_IKS01A1 -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Patterns/Basic" -I../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../../Drivers/BSP/Components/lps22hb -I../../../../../../../../Drivers/BSP/Components/lps25hb -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Drivers/BSP/Components/sx1276" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Drivers/BSP/B-L072Z-LRWAN1"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Middlewares/LoRaWAN/Utilities/trace.o: /run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Utilities/trace.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -DSTM32L072xx -DUSE_B_L072Z_LRWAN1 -DUSE_FULL_LL_DRIVER -DNO_MAC_PRINTF -DREGION_EU868 -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Projects/B-L072Z-LRWAN1/Applications/LoRa/AT_Slave/Core/inc" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Projects/B-L072Z-LRWAN1/Applications/LoRa/AT_Slave/LoRaWAN/App/inc" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Drivers/BSP/CMWX1ZZABZ-0xx" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Drivers/STM32L0xx_HAL_Driver/Inc" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Drivers/CMSIS/Device/ST/STM32L0xx/Include" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Drivers/CMSIS/Include" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Crypto" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Mac" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Phy" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Utilities" -I../../../../../../../../Drivers/BSP/X_NUCLEO_IKS01A1 -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Patterns/Basic" -I../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../../Drivers/BSP/Components/lps22hb -I../../../../../../../../Drivers/BSP/Components/lps25hb -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Drivers/BSP/Components/sx1276" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Drivers/BSP/B-L072Z-LRWAN1"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Middlewares/LoRaWAN/Utilities/utilities.o: /run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Utilities/utilities.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -DSTM32L072xx -DUSE_B_L072Z_LRWAN1 -DUSE_FULL_LL_DRIVER -DNO_MAC_PRINTF -DREGION_EU868 -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Projects/B-L072Z-LRWAN1/Applications/LoRa/AT_Slave/Core/inc" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Projects/B-L072Z-LRWAN1/Applications/LoRa/AT_Slave/LoRaWAN/App/inc" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Drivers/BSP/CMWX1ZZABZ-0xx" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Drivers/STM32L0xx_HAL_Driver/Inc" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Drivers/CMSIS/Device/ST/STM32L0xx/Include" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Drivers/CMSIS/Include" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Crypto" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Mac" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Phy" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Utilities" -I../../../../../../../../Drivers/BSP/X_NUCLEO_IKS01A1 -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Patterns/Basic" -I../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../../Drivers/BSP/Components/lps22hb -I../../../../../../../../Drivers/BSP/Components/lps25hb -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Drivers/BSP/Components/sx1276" -I"/run/media/martino/work/STM32CubeExpansion_LRWAN_V1.3.1/Drivers/BSP/B-L072Z-LRWAN1"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

