
#include "../HAL/HAL_XPCC_Class.h"

//#include <AP_HAL/AP_HAL.h>

using namespace XpccHAL;

extern UARTDriver uartADriver;
extern UARTDriver uartBDriver;
extern UARTDriver uartCDriver;
extern UARTDriver uartDDriver;
extern UARTDriver uartEDriver;
extern UARTDriver uartConsoleDriver;

//Semaphore  i2cSemaphore;
//I2CDriver  i2cDriver(&i2cSemaphore);
//SPIDeviceManager spiDeviceManager;
XpccHAL::AnalogIn analogIn;
Storage storageDriver;
GPIO gpioDriver;
RCInput rcinDriver;
RCOutput rcoutDriver;
Scheduler schedulerInstance;
Util utilInstance;
XpccHAL::I2CDeviceManager i2cdev;

HAL_XPCC::HAL_XPCC() :
    AP_HAL::HAL(
        &uartADriver,
        &uartBDriver,
        &uartCDriver,
        &uartDDriver,
        &uartEDriver,
		0, //uartF
		&i2cdev, //AP_HAL::I2CDeviceManager* _i2c_mgr,
        0, // AP_HAL::SPIDeviceManager* _spi,
        &analogIn,
        &storageDriver,
        &uartConsoleDriver,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance,
		0 //AP_HAL::OpticalFlow
		)
{}

namespace AP_HAL {

const HAL& get_HAL() {
	static const HAL_XPCC hal;
	return hal;
}

}

//const AP_HAL::HAL& hal = AP_HAL::get_HAL();

