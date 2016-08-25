/*
 * DFU.hpp
 *
 *  Created on: Aug 24, 2016
 *      Author: walmis
 */

#pragma once

#include <xpcc/driver/connectivity/usb/USBDevice/DFU.hpp>

class DFU final : public xpcc::DFUHandler {
public:
	void do_detach() {
		dfu_detach = true;
	}
	static volatile bool dfu_detach;
};

