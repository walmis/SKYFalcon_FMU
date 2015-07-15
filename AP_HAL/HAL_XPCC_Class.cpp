
#include <AP_HAL.h>

#include "HAL_XPCC_Class.h"
#include "AP_HAL_Empty_Private.h"

using namespace XpccHAL;

extern UARTDriver uartADriver;
extern UARTDriver uartBDriver;
extern UARTDriver uartCDriver;
extern UARTDriver uartDDriver;
extern UARTDriver uartEDriver;
extern UARTDriver uartConsoleDriver;

Semaphore  i2cSemaphore;
I2CDriver  i2cDriver(&i2cSemaphore);
SPIDeviceManager spiDeviceManager;
AnalogIn analogIn;
Storage storageDriver;
GPIO gpioDriver;
RCInput rcinDriver;
RCOutput rcoutDriver;
Scheduler schedulerInstance;
Util utilInstance;

HAL_XPCC::HAL_XPCC() :
    AP_HAL::HAL(
        &uartADriver,
        &uartBDriver,
        &uartCDriver,
        &uartDDriver,
        &uartEDriver,
        &i2cDriver,
		nullptr,
		nullptr,
        &spiDeviceManager,
        &analogIn,
        &storageDriver,
        &uartConsoleDriver,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance)
{}

void HAL_XPCC::init(int argc,char* const argv[]) const {
    scheduler->init(0);

    analogin->init(0);

    rcout->init(0);
    rcin->init(0);

    i2c->begin();

    storage->init(0);
}

const HAL_XPCC AP_HAL_XPCC;

