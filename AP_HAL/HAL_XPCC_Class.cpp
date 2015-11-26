#include <AP_HAL.h>

#include "HAL_XPCC_Class.h"
#include "AP_HAL_Empty_Private.h"

#include "../pindefs.hpp"
#include <xpcc/architecture.hpp>

//defined in main
extern volatile bool dfu_detach;

using namespace XpccHAL;

extern UARTDriver uartADriver;
extern UARTDriver uartBDriver;
extern UARTDriver uartCDriver;
extern UARTDriver uartDDriver;
extern UARTDriver uartEDriver;
extern UARTDriver uartConsoleDriver;

Semaphore  i2cSemaphore;
I2CDriver  i2cDriver(&i2cSemaphore);
SPIDeviceManager spiDeviceManager;
AnalogIn analogIn;
Storage storageDriver;
GPIO gpioDriver;
RCInput rcinDriver;
RCOutput rcoutDriver;
Scheduler schedulerInstance;
Util utilInstance;

HAL_XPCC::HAL_XPCC() :
    AP_HAL::HAL(
        &uartADriver,
        &uartBDriver,
        &uartCDriver,
        &uartDDriver,
        &uartEDriver,
        &i2cDriver,
		nullptr,
		nullptr,
        &spiDeviceManager,
        &analogIn,
        &storageDriver,
        &uartConsoleDriver,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance)
{}


const AP_HAL::HAL& AP_HAL::get_HAL() {
	static const HAL_XPCC AP_HAL_XPCC;
	return AP_HAL_XPCC;
}

namespace AP_HAL {

void init()
{
}

void panic(const char *errormsg, ...) {
	XPCC_LOG_ERROR << errormsg << xpcc::endl;
	AP_HAL::get_HAL().console->println(errormsg);
    LedBlue::set();
    LedRed::set();
    LedGreen::set();
    for(;;) {

		if(dfu_detach) {
			xpcc::sleep(100);
			AP_HAL::get_HAL().scheduler->reboot(true);
		}
    }
}

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
}
