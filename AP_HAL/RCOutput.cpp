#include <xpcc/architecture.hpp>
#include "RCOutput.h"
#include <stdio.h>
#include "../PWM_Outputs.hpp"

using namespace XpccHAL;

#define TIMER_PRESCALE 10

PWM_Outputs pwm;

void RCOutput::init(void* machtnichts) {
	pwm.init();
}

void RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz) {
	freq = freq_hz;
	pwm.setFrequency(freq);
}

uint16_t RCOutput::get_freq(uint8_t ch) {
	return freq;
}

void RCOutput::enable_ch(uint8_t ch)
{

}

void RCOutput::disable_ch(uint8_t ch)
{

}

void RCOutput::write(uint8_t ch, uint16_t period_us)
{
	_channels[ch] = period_us;
	pwm.setOutput(ch, period_us);
}

void RCOutput::write(uint8_t ch, uint16_t* period_us, uint8_t len)
{
    for (int i = 0; i < len; i++) {
        write(i + ch, period_us[i]);
    }
}

uint16_t RCOutput::read(uint8_t ch) {
    return _channels[ch];
}

void RCOutput::read(uint16_t* period_us, uint8_t len)
{
	for(int i = 0; i < len; i++) {
		period_us[i] = _channels[i];
	}
}

