
#ifndef __AP_HAL_EMPTY_SEMAPHORE_H__
#define __AP_HAL_EMPTY_SEMAPHORE_H__

#include "AP_HAL_XPCC.h"
#include <xpcc/processing/rtos.hpp>

class XpccHAL::Semaphore final : public AP_HAL::Semaphore {
public:
    Semaphore() : sem(false) {}
    bool give();
    bool take(uint32_t timeout_ms);
    bool take_nonblocking();
private:

    chibios_rt::BinarySemaphore sem;
};

#endif // __AP_HAL_EMPTY_SEMAPHORE_H__
