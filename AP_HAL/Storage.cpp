#include <stdio.h>
#include <string.h>
#include "Storage.h"
//#include <eeprom/eeprom.hpp>
#include <xpcc/container/linked_list.hpp>
#include <xpcc/architecture.hpp>
#include <xpcc/driver/storage/i2c_eeprom.hpp>
using namespace XpccHAL;
using namespace xpcc;

extern const AP_HAL::HAL& hal;

I2cEeprom<xpcc::stm32::I2cMaster1> eeprom(0x50, 2, 32);

uint8_t eeprom_block[8192];

struct WriteOperation {
	uint16_t	address;
	uint8_t		count;
};

xpcc::LinkedList<WriteOperation> op_list;

Storage::Storage()
{
}

void Storage::init(void*)
{
	//read 8k eeprom to memory
	for(uint32_t i = 0; i < 8192; i+=128) {
		if(!eeprom.read(i, &eeprom_block[i], 128)) {
			AP_HAL::panic("PANIC: Eeprom init failed\n");
		}
	}

	start(NORMALPRIO-1);
}

void Storage::read_block(void* dst, uint16_t addr, size_t n) {
//	if(xpcc::isInterruptContext()) {
//		hal.scheduler->panic("PANIC: eeprom read from ISR\n");
//	}
	//XPCC_LOG_DEBUG .printf("r %d %d\n", src, n);
	if(addr+n > 8192) {
		return;
	}
	memcpy(dst, &eeprom_block[addr], n);

}

void Storage::main() {
	chibios_rt::BaseThread::setName("EEStorage");
	while(1) {
		dataEvt.wait(100);

		while(!op_list.isEmpty()) {
			WriteOperation op = op_list.getFront();
			eeprom.write(op.address, &eeprom_block[op.address], op.count);
			eeprom_lock.lock();
			op_list.removeFront();
			eeprom_lock.unlock();
		}
	}
}

void Storage::write_block(uint16_t loc, const void* src, size_t n)
{
	if(loc+n > 8192) return;

	memcpy(&eeprom_block[loc], src, n);

	WriteOperation op;
	op.address = loc;
	op.count = n;

	eeprom_lock.lock();
	op_list.append(op);
	eeprom_lock.unlock();

	dataEvt.signal();

}
