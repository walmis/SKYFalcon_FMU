
#ifndef __AP_HAL_EMPTY_RCOUTPUT_H__
#define __AP_HAL_EMPTY_RCOUTPUT_H__

#include "../HAL/AP_HAL_XPCC.h"

constexpr int num_channels = 9;

class XpccHAL::RCOutput : public AP_HAL::RCOutput {
    void     init();
    void     set_freq(uint32_t chmask, uint16_t freq_hz);
    uint16_t get_freq(uint8_t ch);
    void     enable_ch(uint8_t ch);
    void     disable_ch(uint8_t ch);
    void     write(uint8_t ch, uint16_t period_us);
    void     write(uint8_t ch, uint16_t* period_us, uint8_t len);
    uint16_t read(uint8_t ch);
    void     read(uint16_t* period_us, uint8_t len);

    void     cork();
    void 	 push();

private:

    uint32_t prescale;
    uint16_t freq;
    uint16_t _channels[num_channels];
};

#endif // __AP_HAL_EMPTY_RCOUTPUT_H__
