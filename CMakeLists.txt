cmake_minimum_required(VERSION 3.5)
include(ExternalProject)

SET(XPCC_DIR ${CMAKE_SOURCE_DIR}/../xpcc/)
SET(PLATFORM "stm32f4")
SET(PROJECT_NAME "SKYFalcon_FMU")
#SET(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/stm32/linkscript.ld")
SET(XPCC_CMAKE_DEFINES "-DCHIBI_RTOS=1")
SET(LIBRARIES "${CMAKE_SOURCE_DIR}/ardupilot/build/libArduCopter.a") #additional libraries

if(NOT DEFINED ${FRAME})
SET(FRAME quad)
endif()

SET(ARDUPILOT_MAKE $(MAKE) -C ArduCopter skyfalcon-${FRAME} BUILDROOT=${CMAKE_SOURCE_DIR}/ardupilot/build)

ExternalProject_Add(ardupilot
    SOURCE_DIR ${CMAKE_SOURCE_DIR}/../ardupilot
    CONFIGURE_COMMAND ""
    BUILD_IN_SOURCE 1
    PREFIX ardupilot
    BUILD_COMMAND ${ARDUPILOT_MAKE}
    INSTALL_COMMAND ""
)

add_definitions(
	-DCONFIG_HAL_BOARD=HAL_BOARD_SKYFALCON
)

include_directories(
../ardupilot/ArduCopter
../ardupilot/libraries
./ardupilot/build/libraries/GCS_MAVLink/
)

add_custom_target(
   ardupilot_make
   COMMAND ${ARDUPILOT_MAKE}
   WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/../ardupilot
)

set(SOURCES
	./radio.cpp
	./ChibiOS/chbAlarm.cpp
	./lowlevel.cpp
	./HAL/system.cpp
	./HAL/UARTDriver.cpp
	./HAL/AnalogIn.cpp
	#./AP_HAL/SPIDriver.cpp
	#./AP_HAL/Semaphores.cpp
	./HAL/RCInput.cpp
	./HAL/GPIO.cpp
	./HAL/DataFlash_Xpcc.cpp
	./HAL/RCOutput.cpp
	./HAL/Storage.cpp
	./HAL/Util.cpp
	#./AP_HAL/PrivateMember.cpp
	./HAL/HAL_XPCC_Class.cpp
	./HAL/Scheduler.cpp
	#./AP_HAL/I2CDriver.cpp
	#./AP_HAL/Storage_cache.cpp
	./rh_compat.cpp
	./fault.cpp
	./time.cpp
	./main.cpp
	./dfu.cpp
	./AP_Radio.cpp
	./ChibiOS/malloc.c
	./stm32/startup.c
	./stm32/system_stm32f4xx.c
	./stm32/stm32f4xx_flash.c
)

include(${XPCC_DIR}/xpccProject.cmake)

project(${PROJECT_NAME} CXX C)

set(CMAKE_INCLUDE_SYSTEM_FLAG_CXX "-I")
set(CMAKE_INCLUDE_SYSTEM_FLAG_C "-I")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-math-errno")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-math-errno")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fno-math-errno")

add_custom_command(TARGET ${PROJECT_NAME}  POST_BUILD
                        COMMAND cp
                        ARGS "${PROJECT_NAME}.bin" "${PROJECT_NAME}.dfu"
                        COMMAND dfu-suffix
                        ARGS    -v ffff -p 32fc -a "${PROJECT_NAME}.dfu"
                        COMMENT "Generating DFU Image"
                        VERBATIM)
                        
add_dependencies(xpcc ardupilot ardupilot_make)
