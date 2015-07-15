/*
 * AP_Radio.hpp
 *
 *  Created on: Jun 6, 2015
 *      Author: walmis
 */

#ifndef AP_RADIO_HPP_
#define AP_RADIO_HPP_

#include <AP_Param.h>

class AP_Radio {
public:
	AP_Radio() {
		AP_Param::setup_object_defaults(this, var_info);
	}

	static const struct AP_Param::GroupInfo var_info[];
//protected:
	AP_Int32 frequency;
	AP_Int8 txPower;

	AP_Int8 modemCfg;
	AP_Int8 fhChannels;
	AP_Int16 maxFragment;
};

void logRadioError(uint8_t error_code);

extern AP_Radio radio_cfg;

#endif /* AP_RADIO_HPP_ */
