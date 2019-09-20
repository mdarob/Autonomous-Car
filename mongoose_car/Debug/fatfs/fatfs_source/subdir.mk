################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../fatfs/fatfs_source/diskio.c \
../fatfs/fatfs_source/ff.c \
../fatfs/fatfs_source/ffsystem.c \
../fatfs/fatfs_source/ffunicode.c \
../fatfs/fatfs_source/fsl_sd_disk.c \
../fatfs/fatfs_source/fsl_sdspi_disk.c 

OBJS += \
./fatfs/fatfs_source/diskio.o \
./fatfs/fatfs_source/ff.o \
./fatfs/fatfs_source/ffsystem.o \
./fatfs/fatfs_source/ffunicode.o \
./fatfs/fatfs_source/fsl_sd_disk.o \
./fatfs/fatfs_source/fsl_sdspi_disk.o 

C_DEPS += \
./fatfs/fatfs_source/diskio.d \
./fatfs/fatfs_source/ff.d \
./fatfs/fatfs_source/ffsystem.d \
./fatfs/fatfs_source/ffunicode.d \
./fatfs/fatfs_source/fsl_sd_disk.d \
./fatfs/fatfs_source/fsl_sdspi_disk.d 


# Each subdirectory must supply rules for building sources it contributes
fatfs/fatfs_source/%.o: ../fatfs/fatfs_source/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -O0 -g3 -Wall -c -fmessage-length=0 -mcpu=cortex-m7 -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


