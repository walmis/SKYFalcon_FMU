cmake_minimum_required(VERSION 3.5)
include(ExternalProject)

SET(XPCC_DIR ${CMAKE_SOURCE_DIR}/../xpcc/)
SET(PLATFORM "stm32f4")
SET(PROJECT_NAME "SKYFalcon_FMU")
#SET(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/stm32/linkscript.ld")
SET(XPCC_CMAKE_DEFINES "-DCHIBI_RTOS=1")
SET(LIBRARIES "${CMAKE_BINARY_DIR}/ardupilot/build/libArduCopter.a") #additional libraries

set(USB_PRODUCT_ID 0x32fc)
set(USB_VENDOR_ID 0xffff)

if(NOT DEFINED ${FRAME})
SET(FRAME quad)
endif()

SET(ARDUPILOT_MAKE $(MAKE) -C ArduCopter skyfalcon-${FRAME} BUILDROOT=${CMAKE_BINARY_DIR}/ardupilot/build)

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
${CMAKE_BINARY_DIR}/ardupilot/build/libraries/GCS_MAVLink/
)

add_custom_target(
   ardupilot_make
   COMMAND ${ARDUPILOT_MAKE}
   WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/../ardupilot
)

add_custom_target(
   ardupilot_clean
   COMMAND ${ARDUPILOT_MAKE} clean
   WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/../ardupilot
)

set(SOURCES
	./radio.cpp
	./radio_lowlevel.cpp
	./lowlevel.cpp
	./HAL/system.cpp
	./HAL/UARTDriver.cpp
	./HAL/AnalogIn.cpp
	#./AP_HAL/SPIDriver.cpp
	./HAL/Semaphores.cpp
	./HAL/RCInput.cpp
	./HAL/GPIO.cpp
	./HAL/DataFlash_Xpcc.cpp
	./HAL/RCOutput.cpp
	./HAL/Storage.cpp
	./HAL/Util.cpp
	#./AP_HAL/PrivateMember.cpp
	./HAL/HAL_XPCC_Class.cpp
	./HAL/Scheduler.cpp
	./HAL/I2CDevice.cpp
	#./AP_HAL/Storage_cache.cpp
	./rh_compat.cpp
	./fault.cpp
	./time.cpp
	./main.cpp
	./dfu.cpp
	./AP_Radio.cpp
	./stm32/startup.c
	./stm32/system_stm32f4xx.c
	#./stm32/stm32f4xx_flash.c
)

execute_process(COMMAND
  git describe --match=NeVeRmAtCh --always --abbrev=8 --dirty
  WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
  OUTPUT_VARIABLE SKYFALCON_GITVER
  ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
  
execute_process(COMMAND
  git describe --match=NeVeRmAtCh --always --abbrev=8 --dirty
  WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/../ardupilot"
  OUTPUT_VARIABLE ARDUPILOT_GITVER
  ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
  

include(${XPCC_DIR}/xpccProject.cmake)

project(${PROJECT_NAME} CXX C)

set(CMAKE_INCLUDE_SYSTEM_FLAG_CXX "-I")
set(CMAKE_INCLUDE_SYSTEM_FLAG_C "-I")

add_subdirectory(chibi)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-math-errno -Wno-non-virtual-dtor")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-math-errno")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fno-math-errno")
add_definitions(-DSKYFALCON_GITVER="${SKYFALCON_GITVER}" -DARDUPILOT_GITVER="${ARDUPILOT_GITVER}")
MESSAGE("Git version ${SKYFALCON_GITVER}")
add_custom_command(TARGET ${PROJECT_NAME}  POST_BUILD
                        COMMAND cp
                        ARGS "${PROJECT_NAME}.bin" "${PROJECT_NAME}.dfu"
                        COMMAND dfu-suffix
                        ARGS    -v ffff -p 32fc -a "${PROJECT_NAME}.dfu"
                        COMMENT "Generating DFU Image"
                        VERBATIM)
                        
add_dependencies(ardupilot_make ardupilot)
add_dependencies(${PROJECT_NAME} ardupilot ardupilot_make)
