
#ifndef __AP_HAL_EMPTY_STORAGE_H__
#define __AP_HAL_EMPTY_STORAGE_H__

#include "AP_HAL_XPCC.h"
#include <xpcc/debug.hpp>
#include <xpcc/processing.hpp>

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
	static const uint16_t maxNodes = 25;

	Node array[maxNodes];
};

class XpccHAL::Storage final : public AP_HAL::Storage, chibios_rt::BaseStaticThread<256> {
public:
    Storage();
    void init(void *);
    void read_block(void *dst, uint16_t src, size_t n);
    void write_block(uint16_t dst, const void* src, size_t n);

    void main() override;

    Cache block_cache;
    chibios_rt::Mutex eeprom_lock;
    xpcc::Event dataEvt;
};

#endif // __AP_HAL_EMPTY_STORAGE_H__
