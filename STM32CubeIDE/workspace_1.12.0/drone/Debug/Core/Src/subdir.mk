################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/accel.c \
../Core/Src/attitude.c \
../Core/Src/baro.c \
../Core/Src/comm.c \
../Core/Src/control_motor.c \
../Core/Src/filter.c \
../Core/Src/flight_math.c \
../Core/Src/freertos.c \
../Core/Src/gpio.c \
../Core/Src/gyro.c \
../Core/Src/i2c.c \
../Core/Src/imu.c \
../Core/Src/lps22hb_interface.c \
../Core/Src/lps22hb_reg.c \
../Core/Src/lsm303agr_interface.c \
../Core/Src/lsm303agr_reg.c \
../Core/Src/lsm6dsl_interface.c \
../Core/Src/lsm6dsl_reg.c \
../Core/Src/mag.c \
../Core/Src/main.c \
../Core/Src/pid.c \
../Core/Src/sensor_align.c \
../Core/Src/sensors.c \
../Core/Src/stm32f7xx_hal_msp.c \
../Core/Src/stm32f7xx_hal_timebase_tim.c \
../Core/Src/stm32f7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f7xx.c \
../Core/Src/usart.c \
../Core/Src/usb_otg.c 

OBJS += \
./Core/Src/accel.o \
./Core/Src/attitude.o \
./Core/Src/baro.o \
./Core/Src/comm.o \
./Core/Src/control_motor.o \
./Core/Src/filter.o \
./Core/Src/flight_math.o \
./Core/Src/freertos.o \
./Core/Src/gpio.o \
./Core/Src/gyro.o \
./Core/Src/i2c.o \
./Core/Src/imu.o \
./Core/Src/lps22hb_interface.o \
./Core/Src/lps22hb_reg.o \
./Core/Src/lsm303agr_interface.o \
./Core/Src/lsm303agr_reg.o \
./Core/Src/lsm6dsl_interface.o \
./Core/Src/lsm6dsl_reg.o \
./Core/Src/mag.o \
./Core/Src/main.o \
./Core/Src/pid.o \
./Core/Src/sensor_align.o \
./Core/Src/sensors.o \
./Core/Src/stm32f7xx_hal_msp.o \
./Core/Src/stm32f7xx_hal_timebase_tim.o \
./Core/Src/stm32f7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f7xx.o \
./Core/Src/usart.o \
./Core/Src/usb_otg.o 

C_DEPS += \
./Core/Src/accel.d \
./Core/Src/attitude.d \
./Core/Src/baro.d \
./Core/Src/comm.d \
./Core/Src/control_motor.d \
./Core/Src/filter.d \
./Core/Src/flight_math.d \
./Core/Src/freertos.d \
./Core/Src/gpio.d \
./Core/Src/gyro.d \
./Core/Src/i2c.d \
./Core/Src/imu.d \
./Core/Src/lps22hb_interface.d \
./Core/Src/lps22hb_reg.d \
./Core/Src/lsm303agr_interface.d \
./Core/Src/lsm303agr_reg.d \
./Core/Src/lsm6dsl_interface.d \
./Core/Src/lsm6dsl_reg.d \
./Core/Src/mag.d \
./Core/Src/main.d \
./Core/Src/pid.d \
./Core/Src/sensor_align.d \
./Core/Src/sensors.d \
./Core/Src/stm32f7xx_hal_msp.d \
./Core/Src/stm32f7xx_hal_timebase_tim.d \
./Core/Src/stm32f7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f7xx.d \
./Core/Src/usart.d \
./Core/Src/usb_otg.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/accel.cyclo ./Core/Src/accel.d ./Core/Src/accel.o ./Core/Src/accel.su ./Core/Src/attitude.cyclo ./Core/Src/attitude.d ./Core/Src/attitude.o ./Core/Src/attitude.su ./Core/Src/baro.cyclo ./Core/Src/baro.d ./Core/Src/baro.o ./Core/Src/baro.su ./Core/Src/comm.cyclo ./Core/Src/comm.d ./Core/Src/comm.o ./Core/Src/comm.su ./Core/Src/control_motor.cyclo ./Core/Src/control_motor.d ./Core/Src/control_motor.o ./Core/Src/control_motor.su ./Core/Src/filter.cyclo ./Core/Src/filter.d ./Core/Src/filter.o ./Core/Src/filter.su ./Core/Src/flight_math.cyclo ./Core/Src/flight_math.d ./Core/Src/flight_math.o ./Core/Src/flight_math.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/gyro.cyclo ./Core/Src/gyro.d ./Core/Src/gyro.o ./Core/Src/gyro.su ./Core/Src/i2c.cyclo ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c.su ./Core/Src/imu.cyclo ./Core/Src/imu.d ./Core/Src/imu.o ./Core/Src/imu.su ./Core/Src/lps22hb_interface.cyclo ./Core/Src/lps22hb_interface.d ./Core/Src/lps22hb_interface.o ./Core/Src/lps22hb_interface.su ./Core/Src/lps22hb_reg.cyclo ./Core/Src/lps22hb_reg.d ./Core/Src/lps22hb_reg.o ./Core/Src/lps22hb_reg.su ./Core/Src/lsm303agr_interface.cyclo ./Core/Src/lsm303agr_interface.d ./Core/Src/lsm303agr_interface.o ./Core/Src/lsm303agr_interface.su ./Core/Src/lsm303agr_reg.cyclo ./Core/Src/lsm303agr_reg.d ./Core/Src/lsm303agr_reg.o ./Core/Src/lsm303agr_reg.su ./Core/Src/lsm6dsl_interface.cyclo ./Core/Src/lsm6dsl_interface.d ./Core/Src/lsm6dsl_interface.o ./Core/Src/lsm6dsl_interface.su ./Core/Src/lsm6dsl_reg.cyclo ./Core/Src/lsm6dsl_reg.d ./Core/Src/lsm6dsl_reg.o ./Core/Src/lsm6dsl_reg.su ./Core/Src/mag.cyclo ./Core/Src/mag.d ./Core/Src/mag.o ./Core/Src/mag.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/pid.cyclo ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/pid.su ./Core/Src/sensor_align.cyclo ./Core/Src/sensor_align.d ./Core/Src/sensor_align.o ./Core/Src/sensor_align.su ./Core/Src/sensors.cyclo ./Core/Src/sensors.d ./Core/Src/sensors.o ./Core/Src/sensors.su ./Core/Src/stm32f7xx_hal_msp.cyclo ./Core/Src/stm32f7xx_hal_msp.d ./Core/Src/stm32f7xx_hal_msp.o ./Core/Src/stm32f7xx_hal_msp.su ./Core/Src/stm32f7xx_hal_timebase_tim.cyclo ./Core/Src/stm32f7xx_hal_timebase_tim.d ./Core/Src/stm32f7xx_hal_timebase_tim.o ./Core/Src/stm32f7xx_hal_timebase_tim.su ./Core/Src/stm32f7xx_it.cyclo ./Core/Src/stm32f7xx_it.d ./Core/Src/stm32f7xx_it.o ./Core/Src/stm32f7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f7xx.cyclo ./Core/Src/system_stm32f7xx.d ./Core/Src/system_stm32f7xx.o ./Core/Src/system_stm32f7xx.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su ./Core/Src/usb_otg.cyclo ./Core/Src/usb_otg.d ./Core/Src/usb_otg.o ./Core/Src/usb_otg.su

.PHONY: clean-Core-2f-Src

