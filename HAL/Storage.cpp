#include <stdio.h>
#include <string.h>
#include "Storage.h"
//#include <eeprom/eeprom.hpp>
#include <xpcc/container/linked_list.hpp>
#include <xpcc/architecture.hpp>
#include <xpcc/driver/storage/i2c_eeprom.hpp>
#include <new>

using namespace XpccHAL;
using namespace xpcc;

extern const AP_HAL::HAL& hal;

static I2cEeprom<xpcc::stm32::I2cMaster1> eeprom(0x50, 2, 32);

class PendingBlock {
public:
	static void push(uint16_t loc, uint8_t* data, uint8_t size) {
		PendingBlock* b = new (size) PendingBlock;
		b->_size = size;
		b->next = 0;
		b->loc = loc;

		memcpy(b->payload(), data, size);

		if(!base) {
		    base = b;
		} else {
		    auto *t = base;
		    while(t->next) {
		        t = t->next;
		    }
		    t->next = b;
		}
	}

	static AP_HAL::OwnPtr<PendingBlock> pop() {
		PendingBlock* b = base;
		if(b) {
			base = b->next;
		}
		return AP_HAL::OwnPtr<PendingBlock>(b);
	}

	uint8_t* payload() {
	    return (uint8_t*)this+sizeof(PendingBlock);
	}

	uint8_t size() {
	    return _size;
	}

	uint16_t addr() {
		return loc;
	}


	void operator delete(void* ptr) {
	    free(ptr);
	}

private:
    PendingBlock() {}

	void* operator new(std::size_t count, int payload) {
		return calloc(count+payload, 1);
	}

	static PendingBlock* base;

	uint8_t _size;
	uint16_t loc;
	PendingBlock* next = nullptr;
	///.... data
};
PendingBlock* PendingBlock::base = 0;

Storage::Storage()
{
}

void Storage::init()
{

	start(NORMALPRIO-1);
}

void Storage::read_block(void* dst, uint16_t addr, size_t n) {
//	if(xpcc::isInterruptContext()) {
//		hal.scheduler->panic("PANIC: eeprom read from ISR\n");
//	}
	XPCC_LOG_DEBUG .printf("er %d %d\n", addr, n);
	if(addr+n > 8192) {
		return;
	}

	//printf("read %d\n", addr);
	//memcpy(dst, &eeprom_block[addr], n);

	eeprom.read(addr, (uint8_t*)dst, n);
}

void Storage::main() {
	chibios_rt::BaseThread::setName("EEStorage");
	while(1) {
		dataEvt.wait(100);

//		eeprom_lock.lock();
//		op_list.removeFront();
//		eeprom_lock.unlock();

	}
}

void Storage::write_block(uint16_t loc, const void* src, size_t n)
{
	XPCC_LOG_DEBUG .printf("ew %d %d\n", loc, n);
	if(loc+n > 8192) return;

	eeprom_lock.lock();
	PendingBlock::push(loc, (uint8_t*)src, n);
	eeprom_lock.unlock();

	dataEvt.signal();

}
