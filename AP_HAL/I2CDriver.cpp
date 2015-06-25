
#include "AP_HAL_XPCC.h"
#include "I2CDriver.h"
#include <xpcc/architecture.hpp>
#include "../pindefs.hpp"

using namespace XpccHAL;

extern const AP_HAL::HAL& hal;


AP_HAL::Semaphore* I2CDriver::get_semaphore() {
	return _semaphore;
}

void I2CDriver::begin() {
	//hal.scheduler->register_timer_process(AP_HAL_MEMBERPROC(&I2CDriver::watchdog));
	busRelease(true);
}

void I2CDriver::end() {}

void I2CDriver::setTimeout(uint16_t ms) {}
void I2CDriver::setHighSpeed(bool active) {}

#define I2C xpcc::stm32::I2cMaster1
extern const AP_HAL::HAL& hal;

void I2CDriver::busReset() {
	I2C::busReset();
}

void i2stop() {
	Scl::set();
	chibios_rt::BaseThread::sleep(US2ST(10));

    Sda::set();
    chibios_rt::BaseThread::sleep(US2ST(10));
}

void i2start() {
	Sda::set();
	Scl::set();

	chibios_rt::BaseThread::sleep(US2ST(10));

	Sda::reset();
	chibios_rt::BaseThread::sleep(US2ST(10));

	Scl::reset();
	chibios_rt::BaseThread::sleep(US2ST(10));
}

void i2readbit() {
	Sda::set();
	Scl::set();
	chibios_rt::BaseThread::sleep(US2ST(10));

    Scl::reset();
    chibios_rt::BaseThread::sleep(US2ST(10));
}

bool I2CDriver::busRelease(bool force) {
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

bool I2CDriver::startTransaction() {
	uint8_t retry_count = 2;
retry:
	retry_count--;
	if(!I2C::start(this)) {
		//XPCC_LOG_DEBUG << "e1";
		error_count++;
		return 1;
	}

	if(!wait(3)) {
		XPCC_LOG_DEBUG .printf("i2c Timeout (%d, %d)\n", getState(), this->errno);
		XPCC_LOG_DEBUG .printf("I2c: CR1:%x CR2:%x SR1:%x SR2:%x\n", I2C1->CR1, I2C1->CR2, I2C1->SR1, I2C1->SR2);

		XPCC_LOG_DEBUG .printf("Reset st:%d\n", I2C::resetTransaction(this));

		busReset();
		busRelease();

		error_count++;

		if(retry_count) {
			goto retry;
		}
		return 1;
	}

	bool failed = getState() != xpcc::I2cWriteReadTransaction::AdapterState::Idle;
	if(failed)  {

		//XPCC_LOG_DEBUG .printf("e3 %d\n", this->errno);
		if(this->errno == xpcc::I2cMaster::Error::DataNack) {
			if(retry_count) {
				goto retry;
			}
		} else
			error_count++;

		if(this->errno == xpcc::I2cMaster::Error::BusCondition) {
			I2C::busReset();
			busRelease();

			if(retry_count) {
				goto retry;
			}
		}


	}
	return failed;
}

uint8_t I2CDriver::write(uint8_t addr, uint8_t len, uint8_t* data)
{
	if(!initialize(addr, data, len, 0, 0)) {
		error_count++;
		return 1;
	}

	return startTransaction();
}
uint8_t I2CDriver::writeRegister(uint8_t addr, uint8_t reg, uint8_t val)
{
	uint8_t data[2];
	data[0] = reg;
	data[1] = val;
	if(!initialize(addr, data, sizeof(data), 0, 0)) {
		error_count++;
		return 1;
	}
	return startTransaction();
}
uint8_t I2CDriver::writeRegisters(uint8_t addr, uint8_t reg,
                               uint8_t len, uint8_t* data)
{
	uint8_t buf[len + 1];
	buf[0] = reg;
	memcpy(&buf[1], data, len);

	if(!initialize(addr, buf, len+1, 0, 0)) {
		error_count++;
		return 1;
	}
	return startTransaction();
}

uint8_t I2CDriver::read(uint8_t addr, uint8_t len, uint8_t* data)
{
	if(!initialize(addr, 0, 0, data, len)) {
		error_count++;
		return 1;
	}
	return startTransaction();
}

uint8_t I2CDriver::readRegister(uint8_t addr, uint8_t reg, uint8_t* data)
{
	if(!initialize(addr, &reg, 1, data, 1)) {
		error_count++;
		return 1;
	}
	return startTransaction();
}

uint8_t I2CDriver::readRegisters(uint8_t addr, uint8_t reg,
                                      uint8_t len, uint8_t* data)
{
	if(!len) return 1;

	//I2cWriteReadTransaction transaction;

	if(!initialize(addr, &reg, 1, data, len)) {
		error_count++;
		return 1;
	}
	return startTransaction();
}

uint8_t I2CDriver::lockup_count() {
	return error_count;
}

