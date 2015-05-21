
#ifndef __AP_HAL_EMPTY_ANALOGIN_H__
#define __AP_HAL_EMPTY_ANALOGIN_H__

#include "AP_HAL_XPCC.h"
#include <xpcc/math/filter.hpp>

class XpccHAL::AnalogSource final: public AP_HAL::AnalogSource {
public:
    AnalogSource(int16_t chan, AnalogIn* parent);
    float read_average();
    float read_latest();
    void set_pin(uint8_t p);
    void set_stop_pin(uint8_t p);
    void set_settle_time(uint16_t settle_time_ms);
    float voltage_average();
    float voltage_latest();
    float voltage_average_ratiometric() { return voltage_average(); }

private:
    friend class XpccHAL::AnalogIn;
    void _update();

    uint32_t sum;
    uint8_t count;

    float _voltage_latest;
    float _voltage_avg;

    uint8_t _chan;
    int8_t _chan_idx;

    static uint8_t _ch_cnt;
};

class XpccHAL::AnalogIn final: public AP_HAL::AnalogIn {
public:
    AnalogIn();
    void init(void* implspecific);
    AP_HAL::AnalogSource* channel(int16_t n);
    float board_voltage(void);

private:
    friend class AnalogSource;

    void _tick();
    static const uint8_t max_channels = 10;

    uint8_t num_channels;

    AP_HAL::AnalogSource* vdd;
    AP_HAL::AnalogSource* vref;

    XpccHAL::AnalogSource* channels[max_channels];
    uint16_t *samples;

    xpcc::stm32::dma::DMAStream dmastream;
};
#endif // __AP_HAL_EMPTY_ANALOGIN_H__
