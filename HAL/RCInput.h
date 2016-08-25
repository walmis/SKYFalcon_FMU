
#ifndef __AP_HAL_EMPTY_RCINPUT_H__
#define __AP_HAL_EMPTY_RCINPUT_H__

#include <xpcc/architecture.hpp>
#include "../HAL/AP_HAL_XPCC.h"

class XpccHAL::RCInput final : public AP_HAL::RCInput {
public:
    RCInput();
    void init();
    bool  new_input();
    uint8_t num_channels();
    uint16_t read(uint8_t ch);
    uint8_t read(uint16_t* periods, uint8_t len);

    bool set_overrides(int16_t *overrides, uint8_t len);
    bool set_override(uint8_t channel, int16_t override);
    void clear_overrides();

private:
    xpcc::Timestamp last_read;
};

#endif // __AP_HAL_EMPTY_RCINPUT_H__
