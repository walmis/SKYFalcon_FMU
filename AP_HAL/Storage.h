
#ifndef __AP_HAL_EMPTY_STORAGE_H__
#define __AP_HAL_EMPTY_STORAGE_H__

#include "AP_HAL_XPCC.h"

class XpccHAL::Storage final : public AP_HAL::Storage {
public:
    Storage();
    void init(void *);
    void read_block(void *dst, uint16_t src, size_t n);
    void write_block(uint16_t dst, const void* src, size_t n);
};

#endif // __AP_HAL_EMPTY_STORAGE_H__
