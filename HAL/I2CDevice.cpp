
#include "AP_HAL_XPCC.h"
#include <xpcc/architecture.hpp>
#include "../pindefs.hpp"
#include "I2CDevice.h"
#include <chvt.h>
#include <ch.h>

using namespace XpccHAL;

extern const AP_HAL::HAL& hal;

#define I2C xpcc::stm32::I2cMaster1

Semaphore I2CDevice::_semaphore;
thread_t* I2CDevice::bus_thread;
Timer* I2CDevice::timers[NUM_BUS_TIMERS];
uint8_t I2CDevice::registered_timers = 0;


void I2CDevice::busReset() {
	I2C::busReset();
}
//
static void i2stop() {
	Scl::set();
	chibios_rt::BaseThread::sleep(US2ST(10));

    Sda::set();
    chibios_rt::BaseThread::sleep(US2ST(10));
}

static void i2start() {
	Sda::set();
	Scl::set();

	chibios_rt::BaseThread::sleep(US2ST(10));

	Sda::reset();
	chibios_rt::BaseThread::sleep(US2ST(10));

	Scl::reset();
	chibios_rt::BaseThread::sleep(US2ST(10));
}

static void i2readbit() {
	Sda::set();
	Scl::set();
	chibios_rt::BaseThread::sleep(US2ST(10));

    Scl::reset();
    chibios_rt::BaseThread::sleep(US2ST(10));
}
//
bool I2CDevice::bitbangBusRelease(bool force) {
	bool res = true;
	if(!Sda::read() || force) { //sda is low
		res = false;
		Scl::setOutput(true); //prepare to release bus
		Sda::setOutput(true);

		for(int x = 0; x < 5; x++) {
			i2start();
			for(int i = 0; i < 18; i++) {
				i2readbit();
				if(Sda::read()) {
					break;
				}
			}
			i2stop();
			if(Sda::read()) {
				break;
			}
		}

		if(!Sda::read()) {
			XPCC_LOG_DEBUG << "failed to release bus!\n";
		} else {
			res = true;
		}

		Scl::setFunction(xpcc::stm32::AltFunction::AF_I2C1);
		Sda::setFunction(xpcc::stm32::AltFunction::AF_I2C1);

		busReset();
	}

	return res;
}
//
bool I2CDevice::startTransaction() {
	uint8_t retry_count = retries;
retry:
	retry_count--;
	if(!I2C::start(this)) {
		//XPCC_LOG_DEBUG << "e1";
		error_count++;
		return false;
	}

	if(!wait(3)) {
		XPCC_LOG_DEBUG .printf("i2c Timeout (%d, %d)\n", getState(), this->error);
		XPCC_LOG_DEBUG .printf("I2c: CR1:%x CR2:%x SR1:%x SR2:%x\n", I2C1->CR1, I2C1->CR2, I2C1->SR1, I2C1->SR2);

		XPCC_LOG_DEBUG .printf("Reset st:%d\n", I2C::resetTransaction(this));

		busReset();
		bitbangBusRelease();

		error_count++;

		if(retry_count) {
			goto retry;
		}
		return false;
	}

	bool failed = getState() != xpcc::I2cWriteReadTransaction::AdapterState::Idle;
	if(failed)  {

		//XPCC_LOG_DEBUG .printf("e3 %d\n", this->errno);
		if(this->error == xpcc::I2cMaster::Error::DataNack) {
			if(retry_count) {
				goto retry;
			}
		} else
			error_count++;

		if(this->error == xpcc::I2cMaster::Error::BusCondition) {
			I2C::busReset();
			bitbangBusRelease();

			if(retry_count) {
				goto retry;
			}
		}


	}
	return !failed;
}

void I2CDevice::busThread(void* arg) {

	XPCC_LOG_DEBUG .printf("Hello i2c thread\n");
	while(1) {

		eventmask_t ev = chEvtWaitOne(ALL_EVENTS);

		uint8_t event_id = __builtin_ctzl(ev); //count trailing zeroes

		//XPCC_LOG_DEBUG .printf("Event occurred %d\n", event_id);

		Timer* t = timers[event_id];
		if(t) {
			if(!_semaphore.take(1000)) {
				XPCC_LOG_DEBUG .printf("i2c sem timeout\n");
			}

			if(!t->call()) {
				t->stop();
				timers[event_id] = 0;
				delete t;
			}
			_semaphore.give();
		}

		//XPCC_LOG_DEBUG .printf("ev %d\n", chEvtGetAndClearEvents(ALL_EVENTS));

	}
}

I2CDevice::I2CDevice() {
	static void* mem[128];
	if(!bus_thread) {
		bus_thread = chThdCreateStatic(mem, sizeof(mem), NORMALPRIO+1, &busThread, 0);
	}
}

bool I2CDevice::transfer(const uint8_t *send, uint32_t send_len,
						  uint8_t *recv, uint32_t recv_len) {

	if(!initialize(send, send_len, recv, recv_len)) {
		error_count++;
		return false;
	}

	return startTransaction();

}

void I2CDevice::set_address(uint8_t address) {
	initialize(address, 0, 0, 0, 0);
}

bool XpccHAL::I2CDevice::set_speed(Device::Speed speed) {
}

bool XpccHAL::I2CDevice::read_registers_multiple(uint8_t first_reg,
		uint8_t* recv, uint32_t recv_len, uint8_t times) {

	if(!recv_len) return false;

	if(!initialize( &first_reg, 1, recv, recv_len*times)) {
		error_count++;
		return false;
	}
	return startTransaction();
}

AP_HAL::Semaphore* XpccHAL::I2CDevice::get_semaphore() {
	return &_semaphore;
}


void Timer::tmrcb(void* arg) {
	Timer* t = (Timer*)arg;

	chSysLockFromISR();
	chEvtSignalI(t->parent->bus_thread, (1<<t->id));
	chVTSetI((virtual_timer_t*)&t->vt, t->period, tmrcb, t);
	chSysUnlockFromISR();
}




AP_HAL::Device::PeriodicHandle XpccHAL::I2CDevice::register_periodic_callback(
		uint32_t period_usec, Device::PeriodicCb functor) {

	int id = 0;
	for(Timer* t : timers) {
		if(t == NULL) {
			Timer* vt = new Timer(this, id, functor);
			vt->set(US2ST(period_usec));
			timers[id] = vt;
			return (void*)id;
		}

		id++;
	}

	AP_HAL::panic("MAX i2c register_periodic_callbacks reached!");
	return 0;

}

bool XpccHAL::I2CDevice::adjust_periodic_callback(Device::PeriodicHandle h,
		uint32_t period_usec) {
	XPCC_LOG_DEBUG .printf("adjust periodic callback\n");
}
//uint8_t I2CDriver::write(uint8_t addr, uint8_t len, uint8_t* data)
//{
//	if(!initialize(addr, data, len, 0, 0)) {
//		error_count++;
//		return 1;
//	}
//
//	return startTransaction();
//}
//uint8_t I2CDriver::writeRegister(uint8_t addr, uint8_t reg, uint8_t val)
//{
//	uint8_t data[2];
//	data[0] = reg;
//	data[1] = val;
//	if(!initialize(addr, data, sizeof(data), 0, 0)) {
//		error_count++;
//		return 1;
//	}
//	return startTransaction();
//}
//uint8_t I2CDriver::writeRegisters(uint8_t addr, uint8_t reg,
//                               uint8_t len, uint8_t* data)
//{
//	uint8_t buf[len + 1];
//	buf[0] = reg;
//	memcpy(&buf[1], data, len);
//
//	if(!initialize(addr, buf, len+1, 0, 0)) {
//		error_count++;
//		return 1;
//	}
//	return startTransaction();
//}
//
//uint8_t I2CDriver::read(uint8_t addr, uint8_t len, uint8_t* data)
//{
//	if(!initialize(addr, 0, 0, data, len)) {
//		error_count++;
//		return 1;
//	}
//	return startTransaction();
//}
//
//uint8_t I2CDriver::readRegister(uint8_t addr, uint8_t reg, uint8_t* data)
//{
//	if(!initialize(addr, &reg, 1, data, 1)) {
//		error_count++;
//		return 1;
//	}
//	return startTransaction();
//}
//
//uint8_t I2CDriver::readRegisters(uint8_t addr, uint8_t reg,
//                                      uint8_t len, uint8_t* data)
//{
//	if(!len) return 1;
//
//	//I2cWriteReadTransaction transaction;
//
//	if(!initialize(addr, &reg, 1, data, len)) {
//		error_count++;
//		return 1;
//	}
//	return startTransaction();
//}
//
//uint8_t I2CDriver::lockup_count() {
//	return error_count;
//}

