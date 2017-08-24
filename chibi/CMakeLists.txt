

target_include_directories(${PROJECT_NAME} PUBLIC
  rt/include
  common/ports/ARMCMx/
  common/ports/ARMCMx/compilers/GCC/
  ./license
  common/oslib/include/
  .
  ./cpp
)

target_sources(${PROJECT_NAME} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/glue/chbAlarm.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/glue/malloc.c

  ${CMAKE_CURRENT_SOURCE_DIR}/cpp/ch.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/common/ports/ARMCMx/chcore_v7m.c
    ${CMAKE_CURRENT_SOURCE_DIR}/common/ports/ARMCMx/chcore.c
    ${CMAKE_CURRENT_SOURCE_DIR}/common/ports/ARMCMx/compilers/GCC/chcoreasm_v7m.S

    ${CMAKE_CURRENT_SOURCE_DIR}/common/oslib/src/chheap.c
    ${CMAKE_CURRENT_SOURCE_DIR}/common/oslib/src/chmboxes.c
    ${CMAKE_CURRENT_SOURCE_DIR}/common/oslib/src/chmemcore.c
    ${CMAKE_CURRENT_SOURCE_DIR}/common/oslib/src/chmempools.c

    ${CMAKE_CURRENT_SOURCE_DIR}/xpcc/event.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/xpcc/irq_wrapper.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/xpcc/semaphore.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/xpcc/util.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/rt/src/chcond.c
    ${CMAKE_CURRENT_SOURCE_DIR}/rt/src/chdebug.c
    ${CMAKE_CURRENT_SOURCE_DIR}/rt/src/chdynamic.c
    ${CMAKE_CURRENT_SOURCE_DIR}/rt/src/chevents.c
    ${CMAKE_CURRENT_SOURCE_DIR}/rt/src/chmsg.c
    ${CMAKE_CURRENT_SOURCE_DIR}/rt/src/chmtx.c
    ${CMAKE_CURRENT_SOURCE_DIR}/rt/src/chregistry.c
    ${CMAKE_CURRENT_SOURCE_DIR}/rt/src/chschd.c
    ${CMAKE_CURRENT_SOURCE_DIR}/rt/src/chsem.c
    ${CMAKE_CURRENT_SOURCE_DIR}/rt/src/chstats.c
    ${CMAKE_CURRENT_SOURCE_DIR}/rt/src/chsys.c
    ${CMAKE_CURRENT_SOURCE_DIR}/rt/src/chthreads.c
    ${CMAKE_CURRENT_SOURCE_DIR}/rt/src/chtm.c
    ${CMAKE_CURRENT_SOURCE_DIR}/rt/src/chtrace.c
    ${CMAKE_CURRENT_SOURCE_DIR}/rt/src/chvt.c
)

# ADD_LIBRARY(chibi STATIC
#     ${SRCS}
# )
