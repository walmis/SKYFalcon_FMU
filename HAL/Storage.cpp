#include <stdio.h>
#include <string.h>
#include "Storage.h"
//#include <eeprom/eeprom.hpp>
#include <xpcc/container/linked_list.hpp>
#include <xpcc/architecture.hpp>
#include <xpcc/driver/storage/i2c_eeprom.hpp>
#include "I2CDevice.h"
#include <new>

using namespace XpccHAL;
using namespace xpcc;

extern const AP_HAL::HAL& hal;

static I2cEeprom<xpcc::stm32::I2cMaster1> eeprom(0x50, 2, 32);

#define USEBLOCK 1

#ifdef USEBLOCK
uint8_t eeprom_block[8192];
#endif

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
#ifdef USEBLOCK
	for(int i = 0; i < 8192; i+=128) {
		eeprom.read(i, (uint8_t*)eeprom_block+i, 128);
	}
#endif
	start(NORMALPRIO-1);
}

void Storage::read_block(void* dst, uint16_t addr, size_t n) {

	//XPCC_LOG_DEBUG .printf("er %d %d\n", addr, n);
	if(addr+n > 8192) {
		return;
	}

	//printf("read %d\n", addr);
#ifdef USEBLOCK
	memcpy(dst, &eeprom_block[addr], n);
#else
	eeprom_lock.lock();
	eeprom.read(addr, (uint8_t*)dst, n);
	eeprom_lock.unlock();
#endif
#if 0
	uint8_t tmp[n];
	eeprom_lock.lock();
	if(!eeprom.read(addr, tmp, n)) {
		printf("readerr\n");
	}
	eeprom_lock.unlock();
	if(memcmp(tmp, dst, n) != 0) {
		printf("mismatch\n");
	}
#endif
}

void Storage::main() {
	chibios_rt::BaseThread::setName("EEStorage");
	while(1) {
		dataEvt.wait(100);

		eeprom_lock.lock();
		while(1) {
			auto blk = PendingBlock::pop();
			if(blk) {
				//printf("wr %d %d\n", blk->addr(), blk->size());
				//XPCC_LOG_DEBUG .dump_buffer((uint8_t*)blk->payload(), blk->size());
				eeprom.write(blk->addr(), blk->payload(), blk->size() );
			} else {
				break;
			}
		}
		eeprom_lock.unlock();

	}
}

void Storage::write_block(uint16_t loc, const void* src, size_t n)
{
	//XPCC_LOG_DEBUG .printf("ew %d %d\n", loc, n);
	//XPCC_LOG_DEBUG .dump_buffer((uint8_t*)src, n);
	if(loc+n > 8192) return;
#ifdef USEBLOCK
	memcpy(&eeprom_block[loc], src, n);
#endif
	eeprom_lock.lock();
	PendingBlock::push(loc, (uint8_t*)src, n);
	eeprom_lock.unlock();

	dataEvt.signal();

}
