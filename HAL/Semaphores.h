
#ifndef __AP_HAL_EMPTY_SEMAPHORE_H__
#define __AP_HAL_EMPTY_SEMAPHORE_H__

#include <xpcc/processing/rtos_abstraction.hpp>
#include "../HAL/AP_HAL_XPCC.h"

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
