
#include "RCInput.h"
#include "../radio.hpp"

#define NUM_CHANNELS 16

using namespace XpccHAL;
RCInput::RCInput()
{}

void RCInput::init()
{}

#define RC_ROLL 0
#define RC_PITCH 1
#define RC_THROTTLE 2
#define RC_YAW 3
#define RC_5 4
#define RC_6 5
#define RC_7 6
#define RC_8 7
#define RC_9 8

bool RCInput::new_input() {
    if(radio.rcPacketTimestamp != last_read) {
    	return true;
    }
	return false;
}

uint8_t RCInput::num_channels() {
    return NUM_CHANNELS;
}

uint16_t RCInput::read(uint8_t ch) {

	if((xpcc::Clock::now() - radio.rcPacketTimestamp) > 500) {
		radio.rcData.channels[RC_ROLL] = 1500;
		radio.rcData.channels[RC_PITCH] = 1500;
		radio.rcData.channels[RC_YAW] = 1500;
	}

	last_read = radio.rcPacketTimestamp;

	if(ch >= NUM_CHANNELS) {
		return 0;
	}

	return radio.rcData.channels[ch];
}

uint8_t RCInput::read(uint16_t* periods, uint8_t len) {
	for (uint8_t i = 0; i < len; i++){
		periods[i] = read(i);
    }
    return len;
}

bool RCInput::set_overrides(int16_t *overrides, uint8_t len) {
    return true;
}

bool RCInput::set_override(uint8_t channel, int16_t override) {
    return true;
}

void RCInput::clear_overrides()
{}

