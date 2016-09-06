
#ifndef __AP_HAL_EMPTY_I2CDRIVER_H__
#define __AP_HAL_EMPTY_I2CDRIVER_H__

#include <xpcc/architecture.hpp>
#include <xpcc/architecture/peripheral/i2c_adapter.hpp>

#include "../HAL/AP_HAL_XPCC.h"
#include <AP_HAL/HAL.h>
#include <AP_HAL/I2CDevice.h>

class XpccHAL::I2CDevice : public AP_HAL::I2CDevice, xpcc::I2cWriteReadTransaction {
public:

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

private:
    static bool busRelease(bool force = false);
    static void busReset();
    bool startTransaction();

    AP_HAL::Semaphore* _semaphore;
    uint8_t error_count;
    uint8_t retries;
};

class XpccHAL::I2CDeviceManager : public AP_HAL::I2CDeviceManager{
public:
    /* Get a device handle */
    virtual AP_HAL::OwnPtr<AP_HAL::I2CDevice> get_device(uint8_t bus, uint8_t address) {
    	if(bus == 0) {
    		auto dev = new XpccHAL::I2CDevice;
    		dev->set_address(address);
    		return dev;
    	}
    	return nullptr;
    }
};


#endif // __AP_HAL_EMPTY_I2CDRIVER_H__
