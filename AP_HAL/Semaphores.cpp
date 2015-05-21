#include <xpcc/architecture.hpp>
#include "Semaphores.h"

using namespace XpccHAL;

extern const AP_HAL::HAL& hal;

bool Semaphore::give() {
	  volatile unsigned char success = 1;

	  _taken = false;
	  __DMB();
	  __DSB();

	  return success;
}

bool Semaphore::take(uint32_t timeout_ms) {
    if (hal.scheduler->in_timerprocess()) {
        hal.scheduler->panic(PSTR("PANIC: Semaphore::take used from "
                    "inside timer process"));
        return false; /* Never reached - panic does not return */
    }
    return _take_from_mainloop(timeout_ms);
}

bool Semaphore::take_nonblocking() {
    if (hal.scheduler->in_timerprocess()) {
        return _take_nonblocking();
    } else {
        return _take_from_mainloop(0);
    }
}

bool Semaphore::_take_from_mainloop(uint32_t timeout_ms) {
    /* Try to take immediately */
    if (_take_nonblocking()) {
        return true;
    } else if (timeout_ms == 0) {
        /* Return immediately if timeout is 0 */
        return false;
    }

	/* Delay 1ms until we can successfully take, or we timed out */
	xpcc::Timeout<> t(timeout_ms);

	while(!_take_nonblocking()) {
		if(!t.isExpired()) {
			xpcc::yield();
		} else {
			return false; //timed out
		}
	}
	return true;
}

bool Semaphore::_take_nonblocking() {
	  volatile unsigned char failed = 1;
	  register bool lock;

	  lock = __LDREXB((uint8_t*)&_taken);
	  if(!lock) {
		  failed = __STREXB(1, (uint8_t*)&_taken);
		  __DMB();
	  }

	  return !failed;
}
