
#ifndef __AP_HAL_EMPTY_STORAGE_H__
#define __AP_HAL_EMPTY_STORAGE_H__

#include <xpcc/debug.hpp>
#include <xpcc/processing.hpp>
#include "../HAL/AP_HAL_XPCC.h"
#include <ch.hpp>


class XpccHAL::Storage final : public AP_HAL::Storage, chibios_rt::BaseStaticThread<256> {
public:
    Storage();
    void init();
    void read_block(void *dst, uint16_t src, size_t n);
    void write_block(uint16_t dst, const void* src, size_t n);

    void main() override;

    chibios_rt::Mutex eeprom_lock;
    xpcc::Event dataEvt;

    THD_WORKING_AREA(_main_wa, 256);
};

#endif // __AP_HAL_EMPTY_STORAGE_H__
