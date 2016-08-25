#include <xpcc/architecture.hpp>
#include <sys/time.h>

#include "HAL/AP_HAL_XPCC.h"

static uint32_t tset;
static uint32_t timestamp;

extern "C"
int _gettimeofday (struct timeval *tv, void* p) {
	if(timestamp == 0) return -1;
	tv->tv_sec = timestamp + (xpcc::Clock::now().getTime() - tset)/1000;
	return 0;
}

extern "C"
int settimeofday(const struct timeval *tv, const struct timezone *tz) {
	XPCC_LOG_DEBUG .printf("set time %d\n", tv->tv_sec);
	tset = xpcc::Clock::now().getTime();
	timestamp = tv->tv_sec;
	return 0;
}
