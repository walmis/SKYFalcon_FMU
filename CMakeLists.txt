cmake_minimum_required(VERSION 3.5)
include(ExternalProject)

SET(XPCC_DIR ${CMAKE_SOURCE_DIR}/../xpcc/)
SET(PLATFORM "stm32f4")
SET(PROJECT_NAME "SKYFalcon_FMU")
SET(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/stm32/linkscript.ld")
SET(XPCC_CMAKE_DEFINES "-DCHIBI_RTOS=1")
SET(LIBRARIES "${CMAKE_SOURCE_DIR}/../ardupilot/ArduCopter/libArduCopter.a") #additional libraries

if(NOT DEFINED ${FRAME})
SET(FRAME quad)
endif()

SET(ARDUPILOT_MAKE $(MAKE) -C ArduCopter skyfalcon-${FRAME})

ExternalProject_Add(ardupilot
    SOURCE_DIR ${CMAKE_SOURCE_DIR}/../ardupilot
    CONFIGURE_COMMAND ""
    BUILD_IN_SOURCE 1
    BUILD_COMMAND ${ARDUPILOT_MAKE}
    INSTALL_COMMAND ""
)

add_definitions(
	-DCONFIG_HAL_BOARD=HAL_BOARD_SKYFALCON
)

include_directories(
../ardupilot/ArduCopter
../ardupilot/libraries
)

add_custom_target(
   ardupilot_make
   COMMAND ${ARDUPILOT_MAKE}
   WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/../ardupilot
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
	#./AP_HAL/Storage_cache.cpp
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

include(${XPCC_DIR}/xpccProject.cmake)

project(${PROJECT_NAME} CXX C)

set(CMAKE_INCLUDE_SYSTEM_FLAG_CXX "-I")
set(CMAKE_INCLUDE_SYSTEM_FLAG_C "-I")

add_custom_command(TARGET ${PROJECT_NAME}  POST_BUILD
                        COMMAND cp
                        ARGS "${PROJECT_NAME}.bin" "${PROJECT_NAME}.dfu"
                        COMMAND dfu-suffix
                        ARGS    -v ffff -p 32fc -a "${PROJECT_NAME}.dfu"
                        COMMENT "Generating DFU Image"
                        VERBATIM)
                        
add_dependencies(xpcc ardupilot ardupilot_make)
