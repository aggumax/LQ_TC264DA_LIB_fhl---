################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/APP/LQ_ADC_test.c \
../src/APP/LQ_Atom_Motor.c \
../src/APP/LQ_BLDC.c \
../src/APP/LQ_CAMERA.c \
../src/APP/LQ_CCU6_Timer.c \
../src/APP/LQ_EEPROM_TEST.c \
../src/APP/LQ_FFT_TEST.c \
../src/APP/LQ_GPIO_ExInt.c \
../src/APP/LQ_GPIO_KEY.c \
../src/APP/LQ_GPIO_LED.c \
../src/APP/LQ_GPT_mini512.c \
../src/APP/LQ_I2C_9AX.c \
../src/APP/LQ_I2C_VL53.c \
../src/APP/LQ_ICM20602.c \
../src/APP/LQ_IIC_Gyro.c \
../src/APP/LQ_MT9V034.c \
../src/APP/LQ_OLED096.c \
../src/APP/LQ_STM_Timer.c \
../src/APP/LQ_TFT18.c \
../src/APP/LQ_TFT2.c \
../src/APP/LQ_TFTPicFont.c \
../src/APP/LQ_Tim_InputCature.c \
../src/APP/LQ_Tom_Servo.c \
../src/APP/LQ_UART_Bluetooth.c 

O_SRCS += \
../src/APP/LQ_MPU6050_DMP.o 

COMPILED_SRCS += \
./src/APP/LQ_ADC_test.src \
./src/APP/LQ_Atom_Motor.src \
./src/APP/LQ_BLDC.src \
./src/APP/LQ_CAMERA.src \
./src/APP/LQ_CCU6_Timer.src \
./src/APP/LQ_EEPROM_TEST.src \
./src/APP/LQ_FFT_TEST.src \
./src/APP/LQ_GPIO_ExInt.src \
./src/APP/LQ_GPIO_KEY.src \
./src/APP/LQ_GPIO_LED.src \
./src/APP/LQ_GPT_mini512.src \
./src/APP/LQ_I2C_9AX.src \
./src/APP/LQ_I2C_VL53.src \
./src/APP/LQ_ICM20602.src \
./src/APP/LQ_IIC_Gyro.src \
./src/APP/LQ_MT9V034.src \
./src/APP/LQ_OLED096.src \
./src/APP/LQ_STM_Timer.src \
./src/APP/LQ_TFT18.src \
./src/APP/LQ_TFT2.src \
./src/APP/LQ_TFTPicFont.src \
./src/APP/LQ_Tim_InputCature.src \
./src/APP/LQ_Tom_Servo.src \
./src/APP/LQ_UART_Bluetooth.src 

C_DEPS += \
./src/APP/LQ_ADC_test.d \
./src/APP/LQ_Atom_Motor.d \
./src/APP/LQ_BLDC.d \
./src/APP/LQ_CAMERA.d \
./src/APP/LQ_CCU6_Timer.d \
./src/APP/LQ_EEPROM_TEST.d \
./src/APP/LQ_FFT_TEST.d \
./src/APP/LQ_GPIO_ExInt.d \
./src/APP/LQ_GPIO_KEY.d \
./src/APP/LQ_GPIO_LED.d \
./src/APP/LQ_GPT_mini512.d \
./src/APP/LQ_I2C_9AX.d \
./src/APP/LQ_I2C_VL53.d \
./src/APP/LQ_ICM20602.d \
./src/APP/LQ_IIC_Gyro.d \
./src/APP/LQ_MT9V034.d \
./src/APP/LQ_OLED096.d \
./src/APP/LQ_STM_Timer.d \
./src/APP/LQ_TFT18.d \
./src/APP/LQ_TFT2.d \
./src/APP/LQ_TFTPicFont.d \
./src/APP/LQ_Tim_InputCature.d \
./src/APP/LQ_Tom_Servo.d \
./src/APP/LQ_UART_Bluetooth.d 

OBJS += \
./src/APP/LQ_ADC_test.o \
./src/APP/LQ_Atom_Motor.o \
./src/APP/LQ_BLDC.o \
./src/APP/LQ_CAMERA.o \
./src/APP/LQ_CCU6_Timer.o \
./src/APP/LQ_EEPROM_TEST.o \
./src/APP/LQ_FFT_TEST.o \
./src/APP/LQ_GPIO_ExInt.o \
./src/APP/LQ_GPIO_KEY.o \
./src/APP/LQ_GPIO_LED.o \
./src/APP/LQ_GPT_mini512.o \
./src/APP/LQ_I2C_9AX.o \
./src/APP/LQ_I2C_VL53.o \
./src/APP/LQ_ICM20602.o \
./src/APP/LQ_IIC_Gyro.o \
./src/APP/LQ_MT9V034.o \
./src/APP/LQ_OLED096.o \
./src/APP/LQ_STM_Timer.o \
./src/APP/LQ_TFT18.o \
./src/APP/LQ_TFT2.o \
./src/APP/LQ_TFTPicFont.o \
./src/APP/LQ_Tim_InputCature.o \
./src/APP/LQ_Tom_Servo.o \
./src/APP/LQ_UART_Bluetooth.o 


# Each subdirectory must supply rules for building sources it contributes
src/APP/%.src: ../src/APP/%.c src/APP/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -cs --dep-file="$(basename $@).d" --misrac-version=2004 -D__CPU__=tc26xb "-fF:/TC264/十七届平衡单车演示程序/LQ_TC264无刷电机演示程序/LQ_TC264DA_LIB_fhl - 副本/Debug/TASKING_C_C___Compiler-Include_paths.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<" && \
	if [ -f "$(basename $@).d" ]; then sed.exe -r  -e 's/\b(.+\.o)\b/src\/APP\/\1/g' -e 's/\\/\//g' -e 's/\/\//\//g' -e 's/"//g' -e 's/([a-zA-Z]:\/)/\L\1/g' -e 's/\d32:/@TARGET_DELIMITER@/g; s/\\\d32/@ESCAPED_SPACE@/g; s/\d32/\\\d32/g; s/@ESCAPED_SPACE@/\\\d32/g; s/@TARGET_DELIMITER@/\d32:/g' "$(basename $@).d" > "$(basename $@).d_sed" && cp "$(basename $@).d_sed" "$(basename $@).d" && rm -f "$(basename $@).d_sed" 2>/dev/null; else echo 'No dependency file to process';fi
	@echo 'Finished building: $<'
	@echo ' '

src/APP/%.o: ./src/APP/%.src src/APP/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-src-2f-APP

clean-src-2f-APP:
	-$(RM) ./src/APP/LQ_ADC_test.d ./src/APP/LQ_ADC_test.o ./src/APP/LQ_ADC_test.src ./src/APP/LQ_Atom_Motor.d ./src/APP/LQ_Atom_Motor.o ./src/APP/LQ_Atom_Motor.src ./src/APP/LQ_BLDC.d ./src/APP/LQ_BLDC.o ./src/APP/LQ_BLDC.src ./src/APP/LQ_CAMERA.d ./src/APP/LQ_CAMERA.o ./src/APP/LQ_CAMERA.src ./src/APP/LQ_CCU6_Timer.d ./src/APP/LQ_CCU6_Timer.o ./src/APP/LQ_CCU6_Timer.src ./src/APP/LQ_EEPROM_TEST.d ./src/APP/LQ_EEPROM_TEST.o ./src/APP/LQ_EEPROM_TEST.src ./src/APP/LQ_FFT_TEST.d ./src/APP/LQ_FFT_TEST.o ./src/APP/LQ_FFT_TEST.src ./src/APP/LQ_GPIO_ExInt.d ./src/APP/LQ_GPIO_ExInt.o ./src/APP/LQ_GPIO_ExInt.src ./src/APP/LQ_GPIO_KEY.d ./src/APP/LQ_GPIO_KEY.o ./src/APP/LQ_GPIO_KEY.src ./src/APP/LQ_GPIO_LED.d ./src/APP/LQ_GPIO_LED.o ./src/APP/LQ_GPIO_LED.src ./src/APP/LQ_GPT_mini512.d ./src/APP/LQ_GPT_mini512.o ./src/APP/LQ_GPT_mini512.src ./src/APP/LQ_I2C_9AX.d ./src/APP/LQ_I2C_9AX.o ./src/APP/LQ_I2C_9AX.src ./src/APP/LQ_I2C_VL53.d ./src/APP/LQ_I2C_VL53.o ./src/APP/LQ_I2C_VL53.src ./src/APP/LQ_ICM20602.d ./src/APP/LQ_ICM20602.o ./src/APP/LQ_ICM20602.src ./src/APP/LQ_IIC_Gyro.d ./src/APP/LQ_IIC_Gyro.o ./src/APP/LQ_IIC_Gyro.src ./src/APP/LQ_MT9V034.d ./src/APP/LQ_MT9V034.o ./src/APP/LQ_MT9V034.src ./src/APP/LQ_OLED096.d ./src/APP/LQ_OLED096.o ./src/APP/LQ_OLED096.src ./src/APP/LQ_STM_Timer.d ./src/APP/LQ_STM_Timer.o ./src/APP/LQ_STM_Timer.src ./src/APP/LQ_TFT18.d ./src/APP/LQ_TFT18.o ./src/APP/LQ_TFT18.src ./src/APP/LQ_TFT2.d ./src/APP/LQ_TFT2.o ./src/APP/LQ_TFT2.src ./src/APP/LQ_TFTPicFont.d ./src/APP/LQ_TFTPicFont.o ./src/APP/LQ_TFTPicFont.src ./src/APP/LQ_Tim_InputCature.d ./src/APP/LQ_Tim_InputCature.o ./src/APP/LQ_Tim_InputCature.src ./src/APP/LQ_Tom_Servo.d ./src/APP/LQ_Tom_Servo.o ./src/APP/LQ_Tom_Servo.src ./src/APP/LQ_UART_Bluetooth.d ./src/APP/LQ_UART_Bluetooth.o ./src/APP/LQ_UART_Bluetooth.src

.PHONY: clean-src-2f-APP

