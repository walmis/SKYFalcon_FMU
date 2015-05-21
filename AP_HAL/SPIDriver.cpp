
#include "SPIDriver.h"

using namespace XpccHAL;

SPIDeviceDriver::SPIDeviceDriver()
{}

void SPIDeviceDriver::init()
{}

AP_HAL::Semaphore* SPIDeviceDriver::get_semaphore()
{
    return &_semaphore;
}

void SPIDeviceDriver::transaction(const uint8_t *tx, uint8_t *rx, uint16_t len)
{}


void SPIDeviceDriver::cs_assert()
{}

void SPIDeviceDriver::cs_release()
{}

uint8_t SPIDeviceDriver::transfer (uint8_t data)
{
    return 0;
}

void SPIDeviceDriver::transfer (const uint8_t *data, uint16_t len)
{
}

SPIDeviceManager::SPIDeviceManager()
{}

void SPIDeviceManager::init(void *)
{}

AP_HAL::SPIDeviceDriver* SPIDeviceManager::device(enum AP_HAL::SPIDevice)
{
    return &_device;
}

