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

	start(NORMALPRIO-1);
}

void Storage::read_block(void* dst, uint16_t addr, size_t n) {
//	if(xpcc::isInterruptContext()) {
//		hal.scheduler->panic("PANIC: eeprom read from ISR\n");
//	}
	//XPCC_LOG_DEBUG .printf("r %d %d\n", src, n);

	Cache::Node* node = block_cache.get(addr);

	if(node) {
		//XPCC_LOG_DEBUG .printf("hit addr %d\n", addr);
		if(n != node->len) {
			XPCC_LOG_DEBUG .printf("len mismatch\n");
		} else {
			if(node->len > sizeof(node->data)) {
				memcpy(dst, node->dataptr, n);
			} else {
				memcpy(dst, node->data, n);
			}
		}
	} else {
		//XPCC_LOG_DEBUG .printf("miss addr %d\n", addr);
		chibios_rt::LockWith l(eeprom_lock);
		if(!eeprom.read(addr, (uint8_t*)dst, n)) {
			//hal.scheduler->panic("PANIC: Eeprom read failed\n");
			XPCC_LOG_DEBUG .printf("EEPROM read failed (addr %d, st %d)\n", addr, eeprom.errno);
			xpcc::stm32::I2cMaster1::resetTransaction(&eeprom);
		}

		block_cache.put(addr, dst, n);

	}

}

void Storage::main() {
	chibios_rt::BaseThread::setName("EEStorage");
	while(1) {
		dataEvt.wait(100);

		for(int i = 0; i < block_cache.maxNodes; i++) {
			Cache::Node* n = &block_cache.array[i];
			if(n->active && n->write) {
				chibios_rt::LockWith l(eeprom_lock);
				uint8_t* buf;
				if(n->memalloc) {
					buf = (uint8_t*)n->dataptr;
				} else {
					buf = (uint8_t*)n->data;
				}
				//XPCC_LOG_DEBUG .printf("cached write %d len:%d\n", n->addr, n->len);
				if(!eeprom.write(n->addr, buf, n->len)) {
					//hal.scheduler->panic("PANIC: Eeprom write failed\n");
					XPCC_LOG_ERROR .printf("EEPROM write failed\n");
				}
				n->write = false;
			}
		}
	}
}

void Storage::write_block(uint16_t loc, const void* src, size_t n)
{
	//XPCC_LOG_DEBUG .printf("w %d %d\n", loc, n);
	if(!block_cache.putAndWrite(loc, src, n)) {
		chibios_rt::LockWith l(eeprom_lock);
		//XPCC_LOG_DEBUG << "Uncached write\n";
		//block_cache.dump();
		if(!eeprom.write(loc, (uint8_t*)src, n)) {
			//hal.scheduler->panic("PANIC: Eeprom write failed\n");
			XPCC_LOG_ERROR .printf("EEPROM write failed\n");
		}
	} else {
		dataEvt.signal();
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
