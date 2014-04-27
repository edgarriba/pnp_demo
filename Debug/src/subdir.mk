################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/CsvReader.cpp \
../src/CsvWriter.cpp \
../src/ModelRegistration.cpp \
../src/ObjectMesh.cpp \
../src/ObjectModel.cpp \
../src/PnPProblem.cpp \
../src/Utils.cpp \
../src/main.cpp 

OBJS += \
./src/CsvReader.o \
./src/CsvWriter.o \
./src/ModelRegistration.o \
./src/ObjectMesh.o \
./src/ObjectModel.o \
./src/PnPProblem.o \
./src/Utils.o \
./src/main.o 

CPP_DEPS += \
./src/CsvReader.d \
./src/CsvWriter.d \
./src/ModelRegistration.d \
./src/ObjectMesh.d \
./src/ObjectModel.d \
./src/PnPProblem.d \
./src/Utils.d \
./src/main.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/opt/ros/hydro/include -I/opt/ros/hydro/include/opencv -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


