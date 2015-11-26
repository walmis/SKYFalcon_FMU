/*
 * AP_Radio.cpp
 *
 *  Created on: Jun 6, 2015
 *      Author: walmis
 */


#include "AP_Radio.hpp"


#include "Copter.h"

#undef DISABLED
#undef ENABLED
#undef HIGH
#undef LOW
#include "AP_HAL/DataFlash_Xpcc.h"

const AP_Param::GroupInfo AP_Radio::var_info[] {
	    AP_GROUPINFO("FREQUENCY", 0, AP_Radio, frequency, 433000),
	    AP_GROUPINFO("FH_CHANS", 1, AP_Radio, fhChannels, 0),
	    AP_GROUPINFO("MODEM_CFG", 2, AP_Radio, modemCfg, 18),
	    AP_GROUPINFO("MAX_FRAGM", 3, AP_Radio, maxFragment, 96),
	    AP_GROUPINFO("TX_POWER", 4, AP_Radio, txPower, 0),
	    AP_GROUPEND
};


struct PACKED log_Error {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t sub_system;
    uint8_t error_code;
};

void logRadioError(uint8_t error_code) {
    struct log_Error pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ERROR_MSG),
        time_us       : AP_HAL::micros64(),
        sub_system    : ERROR_SUBSYSTEM_RADIO,
        error_code    : error_code,
    };
    dataflash->WriteBlock(&pkt, sizeof(pkt));
}

AP_Radio radio_cfg;
