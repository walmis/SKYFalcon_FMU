#include <stdio.h>
#include <string.h>
#include "Storage.h"
//#include <eeprom/eeprom.hpp>
#include <xpcc/architecture.hpp>
using namespace XpccHAL;

extern const AP_HAL::HAL& hal;

Storage::Storage()
{
}

void Storage::init(void*)
{
//	eeprom.waitAvailable(20);
}

//we are not using semaphore here because eeprom driver uses a
//different i2c delegate than i2cdriver
//and the i2c driver sorts everything out
void Storage::read_block(void* dst, uint16_t src, size_t n) {
//	if(xpcc::isInterruptContext()) {
//		hal.scheduler->panic("PANIC: eeprom read from ISR\n");
//	}
//	if(!eeprom.read(src, (uint8_t*)dst, n)) {
//		hal.scheduler->panic("PANIC: Eeprom read failed\n");
//	}
}

void Storage::write_block(uint16_t loc, const void* src, size_t n)
{
//	if(!eeprom.write(loc, (uint8_t*)src, n)) {
//		hal.scheduler->panic("PANIC: Eeprom write failed\n");
//	}
//	uint8_t buf[n];
//	memset(buf, 0, n);
//	read_block((void*)buf, loc, n);
//	if(memcmp(src, buf, n)) {
//		XPCC_LOG_ERROR .printf("eeprom mismatch loc:%d n:%d\n", loc, n);
//		XPCC_LOG_ERROR .dump_buffer((uint8_t*)buf, n);
//		XPCC_LOG_ERROR << "--- src ---" << xpcc::endl;
//		XPCC_LOG_ERROR .dump_buffer((uint8_t*)src, n);
//	}

}
