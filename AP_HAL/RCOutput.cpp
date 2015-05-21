#include <xpcc/architecture.hpp>
#include "RCOutput.h"
#include <stdio.h>

using namespace XpccHAL;

const uint8_t chMap[] = {1, 2, 3, 4, 5, 6};

#define TIMER_PRESCALE 10

void RCOutput::init(void* machtnichts) {

}

void RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz) {

}

uint16_t RCOutput::get_freq(uint8_t ch) {

}

void RCOutput::enable_ch(uint8_t ch)
{

}

void RCOutput::disable_ch(uint8_t ch)
{

}

void RCOutput::write(uint8_t ch, uint16_t period_us)
{

}

void RCOutput::write(uint8_t ch, uint16_t* period_us, uint8_t len)
{

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

