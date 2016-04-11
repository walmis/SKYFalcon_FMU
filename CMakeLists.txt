cmake_minimum_required(VERSION 3.1)

set(xpcc_DIR, /home/walmis/Desktop/Programming/git/xpcc/build)
set(xpcc_DIR, /home/walmis/Desktop/Programming/git/xpcc/build)
 
project(skyfalcon_fmu C CXX ASM)

find_package(xpcc REQUIRED)
find_package(RadioHead REQUIRED)

SET(ARM_COMMON_FLAGS "-mthumb -mthumb-interwork -ffunction-sections -fdata-sections -g -fno-common -fmessage-length=0")

SET(ASM_OPTIONS "-x assembler-with-cpp")
SET(CMAKE_ASM_FLAGS "${CFLAGS} ${ASM_OPTIONS}" )

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR STM32F407)


SET(CMAKE_C_COMPILER arm-none-eabi-gcc)
SET(CMAKE_CXX_COMPILER arm-none-eabi-g++)
SET(CMAKE_ASM_COMPILER arm-none-eabi-gcc)

SET(COMMON_FLAGS "${ARM_COMMON_FLAGS} -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16")

SET(CMAKE_CXX_FLAGS "${COMMON_FLAGS} -std=gnu++11 -fno-rtti -fno-exceptions")
SET(CMAKE_C_FLAGS "${COMMON_FLAGS} -std=gnu99")
set(CMAKE_EXE_LINKER_FLAGS "-Wl,-gc-sections ")

set(CMAKE_INCLUDE_SYSTEM_FLAG_CXX "-I ")
set(CMAKE_INCLUDE_SYSTEM_FLAG_C "-I ")

if ( xpcc_FOUND )
    message( "xpcc found!")
    
endif()

add_definitions(
	-DCONFIG_HAL_BOARD=HAL_BOARD_SKYFALCON
)

include_directories(
../ardupilot/ArduCopter
../ardupilot/libraries
)

set(SOURCES
	./radio.cpp
	./ChibiOS/chbAlarm.cpp
	./AP_HAL/UARTDriver.cpp
	./AP_HAL/AnalogIn.cpp
	./AP_HAL/SPIDriver.cpp
	./AP_HAL/Semaphores.cpp
	./AP_HAL/RCInput.cpp
	./AP_HAL/GPIO.cpp
	./AP_HAL/DataFlash_Xpcc.cpp
	./AP_HAL/RCOutput.cpp
	./AP_HAL/Storage.cpp
	./AP_HAL/Util.cpp
	./AP_HAL/PrivateMember.cpp
	./AP_HAL/HAL_XPCC_Class.cpp
	./AP_HAL/Scheduler.cpp
	./AP_HAL/I2CDriver.cpp
	./AP_HAL/Storage_cache.cpp
	./rh_compat.cpp
	./fault.cpp
	./time.cpp
	./main.cpp
	./AP_Radio.cpp
	./ChibiOS/malloc.c
	./stm32/startup.c
	./stm32/system_stm32f4xx.c
	./stm32/stm32f4xx_flash.c

)

add_executable(SkyFalcon_FMU ${SOURCES})
target_link_libraries(SkyFalcon_FMU xpcc RadioHead)
