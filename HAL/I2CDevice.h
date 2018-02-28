
#ifndef __AP_HAL_EMPTY_I2CDRIVER_H__
#define __AP_HAL_EMPTY_I2CDRIVER_H__

#include <xpcc/architecture.hpp>
#include <xpcc/architecture/peripheral/i2c_adapter.hpp>
#include <ch.hpp>

#include "../HAL/AP_HAL_XPCC.h"
#include "Semaphores.h"
#include <AP_HAL/HAL.h>
#include <AP_HAL/I2CDevice.h>

//class XpccHAL::I2CDevice;

#define NUM_BUS_TIMERS 6

class Timer {
public:
	Timer(XpccHAL::I2CDevice* parent, int id,
			AP_HAL::Device::PeriodicCb callback) :
			parent(parent), id(id), period(0), callback(callback) {

		chVTObjectInit(&vt);
	}

	void set(uint32_t usec);

	bool call() {
		if(!callback) return false;
		callback();
		return true;
	}

	uint32_t getPeriod() { return period; }
	uint8_t getId() { return id; }

	void start();
	void stop();

private:
	static void tmrcb(void* arg);

	virtual_timer_t vt;
	uint32_t period;
	XpccHAL::I2CDevice* parent;
	uint8_t id;
	AP_HAL::Device::PeriodicCb callback;

};

class XpccHAL::I2CDevice : public AP_HAL::I2CDevice, xpcc::I2cWriteReadTransaction {
public:
	I2CDevice(uint8_t address);
	virtual ~I2CDevice();

	friend Timer;

	/*
	 * Change device address. Note that this is the 7 bit address, it
	 * does not include the bit for read/write.
	 */
	void set_address(uint8_t address);

	/* set number of retries on transfers */
	void set_retries(uint8_t retries) { this->retries = retries; }

	/* Device implementation */

	/* See Device::set_speed() */
	bool set_speed(Device::Speed speed) override;

	/* See Device::transfer() */
	bool transfer(const uint8_t *send, uint32_t send_len,
						  uint8_t *recv, uint32_t recv_len) override;

	/*
	 * Read location from device multiple times, advancing the buffer each
	 * time
	 */
	bool read_registers_multiple(uint8_t first_reg, uint8_t *recv,
										 uint32_t recv_len, uint8_t times);

	/* See Device::get_semaphore() */
	AP_HAL::Semaphore *get_semaphore() override;

	/* See Device::register_periodic_callback() */
	Device::PeriodicHandle register_periodic_callback(
		uint32_t period_usec, Device::PeriodicCb) override;

	/* See Device::adjust_periodic_callback() */
	bool adjust_periodic_callback(
		Device::PeriodicHandle h, uint32_t period_usec) override;

	static bool bitbangBusRelease(bool force = false);

private:
	static void busThread(void*);


    static void busReset();
    bool startTransaction();

    static thread_t* bus_thread;
    static Semaphore _semaphore;
    uint8_t error_count = 0;
    uint8_t retries = 0;
    static Timer* timers[NUM_BUS_TIMERS];
    static uint8_t registered_timers;
};

class XpccHAL::I2CDeviceManager : public AP_HAL::I2CDeviceManager{
public:
    /* Get a device handle */
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> get_device(uint8_t bus, uint8_t address) override;
};


#endif // __AP_HAL_EMPTY_I2CDRIVER_H__
