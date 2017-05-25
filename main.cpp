/*
    SKY Falcon FMU - Copyright (C) 2015 Valmantas Palikša

    This file is part of SKYFalcon FMU project.

    SKYFalcon is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    SKYFalcon is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "HAL/DataFlash_Xpcc.h"
#include "HAL/Storage.h"
#include <xpcc/architecture.hpp>
#include <xpcc/processing.hpp>

#include <xpcc/debug.hpp>
#include <math.h>
#include "lowlevel.hpp"

#include <../ArduCopter/APM_Config.h>
#include <../ArduCopter/Copter.h>

#define DEBUG 1

#define xstr(s) str(s)
#define str(s) #s

#define VERSION "APM:Copter V3.4.6 Build: " __DATE__ " " __TIME__

#define USB_PRODUCT_STRING		"SKY.Falcon FMU:" SKYFALCON_GITVER " Ardupilot:" ARDUPILOT_GITVER " (" xstr(FRAME_CONFIG) ")"
#define USB_MANUFACTURER_STRING	"ENSYS.LT"
#define USB_SERIAL_STRING		"0001"

#define CDC_EPBULK_IN	EP2IN
#define CDC_EPBULK_OUT	EP2OUT
#define CDC_EPINT_IN	EP3IN

#define MSD_EPBULK_IN	EP1IN
#define MSD_EPBULK_OUT	EP1OUT

#include <xpcc/driver/connectivity/usb/USBDevice/Composite/USBCDCMSD.hpp>
#include <xpcc/driver/connectivity/usb/USBDevice/USBSerial/USBSerial.h>
#include <xpcc/driver/connectivity/usb/USBDevice/USBMSD/USBMSD.h>

#include <xpcc/architecture/peripheral/i2c_adapter.hpp>
#include <xpcc/driver/storage/sd/SDCardVolume.hpp>
#include <xpcc/driver/storage/sd/USBMSD_VolumeHandler.hpp>

#include "HAL/AP_HAL_XPCC.h"

#include "PWM_Outputs.hpp"
#include "HAL/UARTDriver.h"
#include "HAL/GPIO.h"
#include "HAL/AnalogIn.h"

#include "pindefs.hpp"
#include "dfu.hpp"

#include "radio.hpp"

#include <ch.hpp>

extern const AP_HAL::HAL& hal;

Radio radio;

BufferedUart<stm32::Usart2> uart2(57600, 256, 128);
BufferedUart<stm32::Usart1> uart1(230400, 256, 128);
BufferedUart<stm32::Usart6> uartGps(57600, 64, 256);

xpcc::log::Logger xpcc::log::debug(uart1);
xpcc::log::Logger xpcc::log::error(uart1);
xpcc::log::Logger xpcc::log::info(uart1);
xpcc::log::Logger xpcc::log::warning(uart1);

SDCardVolume<stm32::SDIO_SDCard> sdCard;

fat::FileSystem fs(&sdCard);

class USBMSD_HandlerWrapper final : public USBMSD_VolumeHandler {
	using USBMSD_VolumeHandler::USBMSD_VolumeHandler;

	int disk_status() override {
		if(!dataflash || dataflash->isWriting()) {
			return NO_DISK;
		} else {
			return USBMSD_VolumeHandler::disk_status();
		}
	}
} msd_handler(&sdCard, 2048);

USBCDCMSD usb(&msd_handler, 0xffff, 0x32fc, 0);
DFU dfu;


void delay(uint32_t ms)
{
    hal.scheduler->delay(ms);
}

void mavlink_delay(uint32_t ms)
{
    hal.scheduler->delay(ms);
}

namespace RH {
uint32_t millis()
{
    return AP_HAL::millis();
}

uint32_t micros()
{
    return AP_HAL::micros();
}
}

XpccHAL::UARTDriver uartADriver(&usb.serial);
XpccHAL::UARTDriver uartBDriver(&uartGps);
#ifndef DEBUG
XpccHAL::UARTDriver uartCDriver(&uart1);
#else
XpccHAL::UARTDriver uartCDriver(0);
#endif
XpccHAL::UARTDriver uartDDriver(&uart2);
XpccHAL::UARTDriver uartEDriver(&radio);
XpccHAL::UARTDriver uartConsoleDriver(&uart1);


void XpccHAL::UARTDriver::setBaud(uint32_t baud, xpcc::IODevice* device) {
	if(device == &uartGps) {
		uartGps.setBaud(baud);
	} else
	if(device == &uart2) {
		uart2.setBaud(baud);
	}
#ifndef DEBUG
	else if(device == &uart1) {
		uart1.setBaud(baud);
	}
#endif
}

bool XpccHAL::GPIO::usb_connected(void)
{
	return !usb.suspended();
}

extern "C"
void _delay_ms(uint32_t ms) {
	Timeout<> t(ms);
	while(!t.isExpired());
}




void dbginit() {
	PB13::setOutput(0);
	PB12::setOutput(0);
}
void dbgset(uint8_t i) {
	if(i == 0) PB12::set();
	else if(i == 1) PB13::set();
}

void dbgclr(uint8_t i) {
	if(i == 0) PB12::reset();
	else if(i == 1) PB13::reset();
}
void dbgtgl(uint8_t i) {
	PB12::toggle();
}

//#define DEBUG



class USBStorage : chibios_rt::BaseStaticThread<512> {
public:
	USBStorage() {
		msd_handler.set_device_strings("ENSYS", "SKY.Falcon", "1.0");

		this->start(NORMALPRIO);
	}
protected:
	void main() {
		chibios_rt::BaseThread::setName("usb_storage");
		while(1) {
			LedRed::reset();
			msd_handler.waitForEvent();
			LedRed::set();
			msd_handler.run();
		}
	}
} usb_storage_task;


//extern AP_HAL::HAL::Callbacks copter;
void HAL_XPCC::run(int argc, char * const argv[], Callbacks* callbacks) const {
	dbginit();
	//Catch NULL pointers
	int region = 0;
	MPU->RNR = region;
	MPU->RBAR = 0;
	MPU->RASR = MPU_RASR_ENABLE_Msk | ((20-1)<<MPU_RASR_SIZE_Pos) | MPU_RASR_SRD_Msk ;

	MPU->CTRL |= MPU_CTRL_ENABLE_Msk | MPU_CTRL_PRIVDEFENA_Msk;
	////

	stm32::SysTickTimer::enable();
	usb.addInterfaceHandler(dfu);

	//wait for devices to settle
	sleep(30);

	//SDIO pins
	SD_LowLevel_init();
	////
	UART_LowLevel_init();
	///
	SPI_LowLevel_init();
	///
	I2C_LowLevel_init();
	///
	LedRed::setOutput();
	LedGreen::setOutput();
	LedBlue::setOutput();
	///
	ExtVoltage::setAnalog();
	ExtCurrent::setAnalog();
	///

//	PB15::setOutputType(GPIOOType::GPIO_OType_OPENDRAIN);
//	PB15::setPullMode(GPIOPuPd::GPIO_PuPd_UP);
//	PB15::setSpeed(GPIOSpeed::GPIO_Speed_100MHz);
	//PB15::setOutput(0);

	XPCC_LOG_DEBUG << "\n\n --- Init Board ---\n\n";
	XPCC_LOG_DEBUG .printf("fAPB1 %d\n", Clocks::getPCLK1Frequency());
	XPCC_LOG_DEBUG .printf("fAPB2 %d\n", Clocks::getPCLK2Frequency());
	XPCC_LOG_DEBUG .printf("fAHB %d\n", Clocks::getHCLKFrequency());

	sdCard.init();

	NVIC_SetPriority(USART1_IRQn, 4);
	NVIC_SetPriority(USART2_IRQn, 4);
	NVIC_SetPriority(USART6_IRQn, 4);

	I2cMaster1::setIrqPriority(5);

	NVIC_SetPriority(OTG_FS_IRQn, 8);
	NVIC_SetPriority(SDIO_IRQn, 8);

	radio.init();

	usb.connect();

	XpccHAL::I2CDevice::bitbangBusRelease();

	//hal.init(0,0);

	hal.scheduler->init();
	hal.storage->init();
	hal.analogin->init();

	callbacks->setup();

	wdt_init();

	for(;;) {
		//dbgset();
		callbacks->loop();
		//loop();
		//dbgtgl();
		//dbgclr();
		sdCard.update();


		if(DFU::dfu_detach) {
			sleep(100);
			hal.scheduler->reboot(true);
		}
	}
}

