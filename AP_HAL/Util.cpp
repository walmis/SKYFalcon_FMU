/*
 * Util.c
 *
 *  Created on: Oct 4, 2014
 *      Author: walmis
 */

#include <xpcc/architecture.hpp>
#include "Util.h"
#include <unistd.h>
#include <sys/time.h>

extern "C" uint8_t __heap_end__;

uint32_t XpccHAL::Util::available_memory(void) {
	size_t mem = chCoreGetStatusX();

	return mem;
}

void XpccHAL::Util::set_system_clock(uint64_t time_utc_usec)  {
	timeval tv;
	tv.tv_sec = time_utc_usec/1000000;
	tv.tv_usec = time_utc_usec % 1000000;

	settimeofday(&tv, 0);
}

