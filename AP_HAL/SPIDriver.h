
#ifndef __AP_HAL_EMPTY_SPIDRIVER_H__
#define __AP_HAL_EMPTY_SPIDRIVER_H__

#include "AP_HAL_XPCC.h"
#include "Semaphores.h"

class XpccHAL::SPIDeviceDriver final : public AP_HAL::SPIDeviceDriver {
public:
    SPIDeviceDriver();
    void init();
    AP_HAL::Semaphore* get_semaphore();
    void transaction(const uint8_t *tx, uint8_t *rx, uint16_t len);

    void cs_assert();
    void cs_release();
    uint8_t transfer (uint8_t data);
    void transfer (const uint8_t *data, uint16_t len);
private:
    Semaphore _semaphore;
};

class XpccHAL::SPIDeviceManager : public AP_HAL::SPIDeviceManager {
public:
    SPIDeviceManager();
    void init(void *);
    AP_HAL::SPIDeviceDriver* device(enum AP_HAL::SPIDevice);
private:
    SPIDeviceDriver _device;
};

#endif // __AP_HAL_EMPTY_SPIDRIVER_H__
