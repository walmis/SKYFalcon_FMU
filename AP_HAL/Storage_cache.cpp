#include <stdio.h>
#include <string.h>
#include "Storage.h"
//#include <eeprom/eeprom.hpp>
#include <xpcc/architecture.hpp>
#include <xpcc/driver/storage/i2c_eeprom.hpp>
using namespace XpccHAL;
using namespace xpcc;

extern const AP_HAL::HAL& hal;

#define CACHE

class Cache {
public:
	struct Node {
		Node() {
			memalloc = 0;
			write = 0;
			len = 0;
			active = 0;
		}
		uint16_t addr = 0;
		uint16_t len : 13;
		uint16_t write : 1;
		uint16_t memalloc : 1;
		uint16_t active : 1;

		uint16_t usedCount = 0;
		union {
			uint8_t* dataptr;
			uint8_t data[4];
		};
	};

	Node* putAndWrite(uint16_t dst, const void* src, size_t len) {
		Node* n = put(dst, src, len);
		if(!n)
			return 0;
		n->write = true;
		return n;
	}

	Node* put(uint16_t dst, const void* src, size_t len) {
		Node* n = get(dst);

		if(!n) {
			if(allocd == maxNodes) {
				removeLeastUsed();
			}

			n = findFreeNode();
			if(n)
				allocd++;
		}
		if(!n) {
			return 0;
		}

		n->addr = dst;
		if(len <= sizeof(n->data)) {
			memcpy(n->data, src, len);
		} else {
			if(n->memalloc) {
				if(n->len != len) {
					delete[] n->dataptr;
					n->dataptr = new uint8_t[len];
				}
			} else {
				n->dataptr = new uint8_t[len];
			}
			memcpy(n->dataptr, src, len);
			n->memalloc = true;
		}
		n->len = len;
		n->usedCount = 0;
		n->active = true;

		return n;
	}

	Node* findFreeNode() {
		for(int i = 0; i < maxNodes; i++) {
			if(!array[i].active) {
				return &array[i];
			}
		}
		return 0;
	}

	Node* get(uint16_t addr) {
		for(int i = 0; i < maxNodes; i++) {
			if(array[i].active && array[i].addr == addr) {
				if(array[i].usedCount != 0xFFFF)
					array[i].usedCount++;
				return &array[i];
			}
		}
		return 0;
	}

	void removeLeastUsed() {
		uint8_t i_min = 0xFF;
		uint16_t minval = 0xFFFF;

		for(uint8_t i = 0; i < maxNodes; i++) {
			if(array[i].active && !array[i].write && array[i].usedCount < minval) {
				minval = array[i].usedCount;
				i_min = i;
			}
		}

		if(i_min != 0xFF) {
			if(array[i_min].memalloc) {
				delete[] array[i_min].dataptr;
				array[i_min].memalloc = false;
			}
			array[i_min].active = false;
			allocd--;
		}
	}

	void dump() {
		for(int i = 0; i < maxNodes; i++) {
			if(array[i].active) {
				XPCC_LOG_DEBUG.printf("%d addr:%d used:%d wr:%d act:%d\n", i, array[i].addr,
						array[i].usedCount,
						array[i].write, array[i].active);
			}
		}

//		printf("base %x last %x\n", first, last);
//
//		while (n) {
//			printf("%x addr:%d len:%x (prev:%x next:%x)\n", n, n->addr, n->len, n->prev, n->next);
//
//			n = n->next;
//		}

	}
//private:
	uint8_t allocd = 0;
	uint16_t count = 0;
	static const uint16_t maxNodes = 200;

	Node array[maxNodes];
};

Cache block_cache;

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
#ifdef CACHE
	start(NORMALPRIO-1);
#endif
}

void Storage::read_block(void* dst, uint16_t addr, size_t n) {
//	if(xpcc::isInterruptContext()) {
//		hal.scheduler->panic("PANIC: eeprom read from ISR\n");
//	}
	//XPCC_LOG_DEBUG .printf("r %d %d\n", src, n);
#ifdef CACHE
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
		XPCC_LOG_DEBUG .printf("miss addr %d\n", addr);
		chibios_rt::LockWith l(eeprom_lock);
#endif
		if(!eeprom.read(addr, (uint8_t*)dst, n)) {
			//hal.scheduler->panic("PANIC: Eeprom read failed\n");
			XPCC_LOG_DEBUG .printf("EEPROM read failed (addr %d, st %d)\n", addr, eeprom.errno);
			xpcc::stm32::I2cMaster1::resetTransaction(&eeprom);
		}
#ifdef CACHE
		block_cache.put(addr, dst, n);

	}
#endif
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
#ifdef CACHE
	if(!block_cache.putAndWrite(loc, src, n)) {
		chibios_rt::LockWith l(eeprom_lock);
		//XPCC_LOG_DEBUG << "Uncached write\n";
		//block_cache.dump();
#endif
		if(!eeprom.write(loc, (uint8_t*)src, n)) {
			//hal.scheduler->panic("PANIC: Eeprom write failed\n");
			XPCC_LOG_ERROR .printf("EEPROM write failed\n");
		}
#ifdef CACHE
	} else {
		dataEvt.signal();
	}
#endif
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
