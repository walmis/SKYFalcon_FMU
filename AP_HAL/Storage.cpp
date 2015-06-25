#include <stdio.h>
#include <string.h>
#include "Storage.h"
//#include <eeprom/eeprom.hpp>
#include <xpcc/architecture.hpp>
#include <xpcc/driver/storage/i2c_eeprom.hpp>
using namespace XpccHAL;
using namespace xpcc;

extern const AP_HAL::HAL& hal;

I2cEeprom<xpcc::stm32::I2cMaster1> eeprom(0x50, 2, 32);

Storage::Storage()
{
}

void Storage::init(void*)
{
//	eeprom.waitAvailable(20);
	//eeprom.isAvailable()

	//sleep(100);
	uint8_t buf;
	if(!eeprom.read(0, (uint8_t*)&buf, 1)) {
		hal.scheduler->panic("PANIC: Eeprom init failed\n");
	}
}

void Storage::read_block(void* dst, uint16_t src, size_t n) {
//	if(xpcc::isInterruptContext()) {
//		hal.scheduler->panic("PANIC: eeprom read from ISR\n");
//	}
	//XPCC_LOG_DEBUG .printf("r %d %d\n", src, n);
	if(!eeprom.read(src, (uint8_t*)dst, n)) {
		//hal.scheduler->panic("PANIC: Eeprom read failed\n");
		XPCC_LOG_DEBUG .printf("EEPROM read failed (addr %d, st %d)\n", src, eeprom.errno);
		xpcc::stm32::I2cMaster1::resetTransaction(&eeprom);
	}
}

void Storage::write_block(uint16_t loc, const void* src, size_t n)
{
	//XPCC_LOG_DEBUG .printf("w %d %d\n", loc, n);
	if(!eeprom.write(loc, (uint8_t*)src, n)) {
		//hal.scheduler->panic("PANIC: Eeprom write failed\n");
		XPCC_LOG_DEBUG .printf("EEPROM write failed\n");
	}
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
