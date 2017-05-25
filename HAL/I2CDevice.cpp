
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
	uint8_t retry_count = 3;
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

extern void dbgset(uint8_t);
extern void dbgclr(uint8_t);

void I2CDevice::busThread(void* arg) {

	XPCC_LOG_DEBUG .printf("Hello i2c thread\n");
	chibios_rt::EvtSource* mpu_ev = static_cast<XpccHAL::Scheduler*>(hal.scheduler)->getSync();

	chibios_rt::EvtListener listener;
	mpu_ev->registerMask(&listener, XpccHAL::Scheduler::MPU_EVENT_MASK);

	while(1) {
		volatile eventmask_t ev = chEvtWaitOneTimeout(XpccHAL::Scheduler::MPU_EVENT_MASK, MS2ST(1));

		for(int i = 0; i < NUM_BUS_TIMERS; i++) {
			Timer* t = timers[i];
			if(t && t->getPeriod() == 1000) {
				chEvtSignal(bus_thread, EVENT_MASK(t->getId()));
			}
		}

		uint32_t timestart = chVTGetSystemTimeX();

		while(1) {
			ev = chEvtWaitOneTimeout(ALL_EVENTS & ~XpccHAL::Scheduler::MPU_EVENT_MASK, 100);
			uint8_t event_id = __builtin_ctzl(ev); //count trailing zeroes

			if(event_id > NUM_BUS_TIMERS) {
				asm("nop");

			} else {
				Timer* tmr = timers[event_id];
				if(tmr) {
					if(!_semaphore.take(1000)) {
						//XPCC_LOG_DEBUG .printf("i2c sem timeout\n");
						continue;
					}

					if(!tmr->call()) {
						tmr->stop();
						timers[event_id] = 0;
						delete tmr;
					}
					_semaphore.give();


				}
			}

			if(chVTGetSystemTimeX() - timestart > 800) break;
		}

	}
}

I2CDevice::I2CDevice(uint8_t address) {
	static void* mem[256];
	if(!bus_thread) {
		bus_thread = chThdCreateStatic(mem, sizeof(mem), NORMALPRIO+1, &busThread, 0);
	}

	set_address(address);
}

I2CDevice::~I2CDevice() {
	asm volatile("nop");
}

bool I2CDevice::transfer(const uint8_t *send, uint32_t send_len,
						  uint8_t *recv, uint32_t recv_len) {

	if(!initialize(send, send_len, recv, recv_len)) {
		error_count++;
		return false;
	}

	bool ret = startTransaction();

	return ret;
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
	chEvtSignalI(t->parent->bus_thread, EVENT_MASK(t->id));
	chVTSetI((virtual_timer_t*)&t->vt, t->period, tmrcb, t);
	chSysUnlockFromISR();
}

void Timer::set(uint32_t usec) {
	static_assert(CH_CFG_ST_FREQUENCY == 1000000,
	                  "CH_CFG_ST_FREQUENCY is not 1000000");
	period = usec;

}

void Timer::start() {
	chVTSet(&vt, period, tmrcb, this);
}

void Timer::stop() {
	chVTReset(&vt);
}

AP_HAL::Device::PeriodicHandle XpccHAL::I2CDevice::register_periodic_callback(
		uint32_t period_usec, Device::PeriodicCb functor) {

	int id = 0;
	for(Timer* t : timers) {
		if(t == NULL) {
			Timer* tmr = new Timer(this, id, functor);

			//1ms is special case, since it is synchronized to MPU DRDY
			//else use virtual timers
			tmr->set(period_usec);

			if(period_usec != 1000) {
				tmr->start();
			}

			timers[id] = tmr;
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

AP_HAL::OwnPtr<AP_HAL::I2CDevice> XpccHAL::I2CDeviceManager::get_device(
		uint8_t bus, uint8_t address)
{
	if(bus == 0) {
		auto dev = AP_HAL::OwnPtr<AP_HAL::I2CDevice>(new XpccHAL::I2CDevice(address));
		return dev;
	} else {
		AP_HAL::panic("Unknown i2c specified");
	}
	return nullptr;
}

