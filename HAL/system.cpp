/*
 * system.cpp
 *
 *  Created on: Aug 24, 2016
 *      Author: walmis
 */

#include <AP_HAL/AP_HAL.h>
#include <xpcc/architecture.hpp>
#include "../pindefs.hpp"
#include "../dfu.hpp"

extern const AP_HAL::HAL& hal;

namespace AP_HAL {


uint32_t millis() {
    return xpcc::Clock::now().getTime();
}

uint32_t micros() {
    return chibios_rt::System::getTimeX();
}

uint64_t millis64() {
    return xpcc::Clock::now().getTime();
}

uint64_t micros64() {
	__disable_irq();
	static uint32_t last_time;
	static uint32_t thigh = 0;
	uint32_t time = chibios_rt::System::getTimeX();
	//handle timer overflow
	if(time < last_time) {
		thigh++;
	}
	last_time = time;

    uint64_t ret = ((uint64_t)thigh)<<32 | time ;
    __enable_irq();
    return ret;
}

void panic(const char *errormsg, ...) {
	XPCC_LOG_ERROR << "PANIC:" << errormsg << xpcc::endl;


	hal.console->print("PANIC:");
    hal.console->println(errormsg);
    //asm("bkpt #0");

    LedBlue::set();
    LedRed::set();
    LedGreen::set();
    for(;;) {

		if(DFU::dfu_detach) {
			xpcc::sleep(100);
			hal.scheduler->reboot(true);
		}
    }
}

}


