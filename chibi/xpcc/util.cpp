#include <xpcc/processing/rtos_abstraction.hpp>
#include <ch.h>

namespace xpcc {
  void yield(uint16_t timeAvailable) {
#ifndef _CHIBIOS_NIL_
  	chThdYield();
#endif
  }

  void sleep(uint16_t time_ms) {
  	chThdSleep(MS2ST(time_ms));
  }

}
