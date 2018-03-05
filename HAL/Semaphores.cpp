#include <xpcc/architecture.hpp>
#include "Semaphores.h"

using namespace XpccHAL;

extern const AP_HAL::HAL& hal;

bool Semaphore::give() {
	sem.signal();
	return true;
}

bool Semaphore::take(uint32_t timeout_ms) {
	if(timeout_ms == HAL_SEMAPHORE_BLOCK_FOREVER) {
		return sem.wait(TIME_INFINITE) == MSG_OK;
	} else {
		return sem.wait(MS2ST(timeout_ms)) == MSG_OK;
	}
}

bool Semaphore::take_nonblocking() {
	return sem.wait(TIME_IMMEDIATE) == MSG_OK;
}
