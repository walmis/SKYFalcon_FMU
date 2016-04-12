cmake_minimum_required(VERSION 3.1)
include(CMakeForceCompiler)

find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif(CCACHE_FOUND)

#set(xpcc_DIR, ../xpcc)
 
find_package(xpcc REQUIRED)
find_package(RadioHead REQUIRED)

SET(ARM_COMMON_FLAGS "-mthumb -mthumb-interwork -ffunction-sections -fdata-sections -g -fno-common -fmessage-length=0")

SET(ASM_OPTIONS "-x assembler-with-cpp")
SET(CMAKE_ASM_FLAGS "${CFLAGS} ${ASM_OPTIONS}" )

    
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

CMAKE_FORCE_C_COMPILER(arm-none-eabi-gcc GNU)
CMAKE_FORCE_CXX_COMPILER(arm-none-eabi-g++ GNU)
SET(CMAKE_ASM_COMPILER arm-none-eabi-gcc)



SET(COMMON_FLAGS "${ARM_COMMON_FLAGS} -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16")

SET(CMAKE_CXX_FLAGS "${COMMON_FLAGS} -std=gnu++11 -fno-rtti -fno-exceptions" CACHE INTERNAL "")
SET(CMAKE_C_FLAGS "${COMMON_FLAGS} -std=gnu99" CACHE INTERNAL "")

set(CMAKE_EXE_LINKER_FLAGS "-mcpu=cortex-m4 -mthumb -Wl,-gc-sections ")

SET(CMAKE_C_COMPILER_ARG1 -mcpu=cortex-m4 -mthumb)
SET(CMAKE_CXX_COMPILER_ARG1 -mcpu=cortex-m4 -mthumb)

project(skyfalcon_fmu)

set(CMAKE_C_COMPILER_ARG1 "")
set(CMAKE_CXX_COMPILER_ARG1 "")

set(CMAKE_INCLUDE_SYSTEM_FLAG_CXX "-I")
set(CMAKE_INCLUDE_SYSTEM_FLAG_C "-I")


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

#target_include_directories(SkyFalcon_FMU PUBLIC $<TARGET_PROPERTY:xpcc,INTERFACE_INCLUDE_DIRECTORIES>)

target_link_libraries(SkyFalcon_FMU PUBLIC xpcc RadioHead)


