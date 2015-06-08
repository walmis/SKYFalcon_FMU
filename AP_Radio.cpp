/*
 * AP_Radio.cpp
 *
 *  Created on: Jun 6, 2015
 *      Author: walmis
 */


#include "AP_Radio.hpp"

const AP_Param::GroupInfo AP_Radio::var_info[] {
	    AP_GROUPINFO("FREQUENCY", 0, AP_Radio, frequency, 433000),
	    AP_GROUPINFO("FH_CHANS", 1, AP_Radio, fhChannels, 4),
	    AP_GROUPINFO("MODEM_CFG", 2, AP_Radio, modemCfg, 18),
	    AP_GROUPINFO("MAX_FRAGM", 3, AP_Radio, maxFragment, 64),
	    AP_GROUPINFO("TX_POWER", 4, AP_Radio, txPower, 0),
	    AP_GROUPEND
};

AP_Radio radio_cfg;
