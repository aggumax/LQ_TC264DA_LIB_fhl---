################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/User/BD_1202V2.c \
../src/User/LQ_AnoScope.c \
../src/User/LQ_Balance.c \
../src/User/LQ_ImageProcess.c \
../src/User/LQ_Inductor.c \
../src/User/LQ_MotorServo.c \
../src/User/LQ_PID.c \
../src/User/Mycode.c 

COMPILED_SRCS += \
./src/User/BD_1202V2.src \
./src/User/LQ_AnoScope.src \
./src/User/LQ_Balance.src \
./src/User/LQ_ImageProcess.src \
./src/User/LQ_Inductor.src \
./src/User/LQ_MotorServo.src \
./src/User/LQ_PID.src \
./src/User/Mycode.src 

C_DEPS += \
./src/User/BD_1202V2.d \
./src/User/LQ_AnoScope.d \
./src/User/LQ_Balance.d \
./src/User/LQ_ImageProcess.d \
./src/User/LQ_Inductor.d \
./src/User/LQ_MotorServo.d \
./src/User/LQ_PID.d \
./src/User/Mycode.d 

OBJS += \
./src/User/BD_1202V2.o \
./src/User/LQ_AnoScope.o \
./src/User/LQ_Balance.o \
./src/User/LQ_ImageProcess.o \
./src/User/LQ_Inductor.o \
./src/User/LQ_MotorServo.o \
./src/User/LQ_PID.o \
./src/User/Mycode.o 


# Each subdirectory must supply rules for building sources it contributes
src/User/%.src: ../src/User/%.c src/User/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -cs --dep-file="$(basename $@).d" --misrac-version=2004 -D__CPU__=tc26xb "-fF:/TC264/十七届平衡单车演示程序/LQ_TC264无刷电机演示程序/LQ_TC264DA_LIB_fhl - 副本/Debug/TASKING_C_C___Compiler-Include_paths.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<" && \
	if [ -f "$(basename $@).d" ]; then sed.exe -r  -e 's/\b(.+\.o)\b/src\/User\/\1/g' -e 's/\\/\//g' -e 's/\/\//\//g' -e 's/"//g' -e 's/([a-zA-Z]:\/)/\L\1/g' -e 's/\d32:/@TARGET_DELIMITER@/g; s/\\\d32/@ESCAPED_SPACE@/g; s/\d32/\\\d32/g; s/@ESCAPED_SPACE@/\\\d32/g; s/@TARGET_DELIMITER@/\d32:/g' "$(basename $@).d" > "$(basename $@).d_sed" && cp "$(basename $@).d_sed" "$(basename $@).d" && rm -f "$(basename $@).d_sed" 2>/dev/null; else echo 'No dependency file to process';fi
	@echo 'Finished building: $<'
	@echo ' '

src/User/%.o: ./src/User/%.src src/User/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-src-2f-User

clean-src-2f-User:
	-$(RM) ./src/User/BD_1202V2.d ./src/User/BD_1202V2.o ./src/User/BD_1202V2.src ./src/User/LQ_AnoScope.d ./src/User/LQ_AnoScope.o ./src/User/LQ_AnoScope.src ./src/User/LQ_Balance.d ./src/User/LQ_Balance.o ./src/User/LQ_Balance.src ./src/User/LQ_ImageProcess.d ./src/User/LQ_ImageProcess.o ./src/User/LQ_ImageProcess.src ./src/User/LQ_Inductor.d ./src/User/LQ_Inductor.o ./src/User/LQ_Inductor.src ./src/User/LQ_MotorServo.d ./src/User/LQ_MotorServo.o ./src/User/LQ_MotorServo.src ./src/User/LQ_PID.d ./src/User/LQ_PID.o ./src/User/LQ_PID.src ./src/User/Mycode.d ./src/User/Mycode.o ./src/User/Mycode.src

.PHONY: clean-src-2f-User

