################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sdmmc/src/fsl_mmc.c \
../sdmmc/src/fsl_sd.c \
../sdmmc/src/fsl_sdio.c \
../sdmmc/src/fsl_sdmmc_common.c \
../sdmmc/src/fsl_sdspi.c 

OBJS += \
./sdmmc/src/fsl_mmc.o \
./sdmmc/src/fsl_sd.o \
./sdmmc/src/fsl_sdio.o \
./sdmmc/src/fsl_sdmmc_common.o \
./sdmmc/src/fsl_sdspi.o 

C_DEPS += \
./sdmmc/src/fsl_mmc.d \
./sdmmc/src/fsl_sd.d \
./sdmmc/src/fsl_sdio.d \
./sdmmc/src/fsl_sdmmc_common.d \
./sdmmc/src/fsl_sdspi.d 


# Each subdirectory must supply rules for building sources it contributes
sdmmc/src/%.o: ../sdmmc/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -O0 -g3 -Wall -c -fmessage-length=0 -mcpu=cortex-m7 -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


