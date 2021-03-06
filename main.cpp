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

#include "AP_HAL/DataFlash_Xpcc.h"
#include "AP_HAL/Storage.h"
#include <xpcc/architecture.hpp>
#include <xpcc/processing.hpp>
#include <xpcc/debug.hpp>
#include <math.h>

#include <../ArduCopter/APM_Config.h>
#include <../ArduCopter/Copter.h>

#define xstr(s) str(s)
#define str(s) #s

#define VERSION "APM:Copter V3.3 Build: " __DATE__ " " __TIME__

#define USB_PRODUCT_STRING		"SKY.Falcon FMU (" VERSION " " xstr(FRAME_CONFIG) ")"
#define USB_MANUFACTURER_STRING	"SKYVideo.pro"
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

#include "AP_HAL/AP_HAL_XPCC.h"

#include "PWM_Outputs.hpp"
#include "AP_HAL/UARTDriver.h"
#include "AP_HAL/GPIO.h"
#include "AP_HAL/AnalogIn.h"

#include "pindefs.hpp"
#include "radio.hpp"

#include <ch.hpp>

BufferedUart<stm32::Usart2> uart2(57600, 256, 128);
BufferedUart<stm32::Usart1> uart1(230400, 256, 128);
BufferedUart<stm32::Usart6> uartGps(57600, 64, 256);

xpcc::log::Logger xpcc::log::debug(uart1);
xpcc::log::Logger xpcc::log::error(uart1);
xpcc::log::Logger xpcc::log::info(uart1);
xpcc::log::Logger xpcc::log::warning(uart1);

SDCardVolume<stm32::SDIO_SDCard> sdCard;

fat::FileSystem fs(&sdCard);

volatile bool dfu_detach;

class USBMSD_HandlerWrapper final : public USBMSD_VolumeHandler {
	using USBMSD_VolumeHandler::USBMSD_VolumeHandler;

	int disk_status() override {
		if(!dataflash || dataflash->isWriting()) {
			return NO_DISK;
		} else {
			return USBMSD_VolumeHandler::disk_status();
		}
	}
};

USBMSD_HandlerWrapper msd_handler(&sdCard, 2048);
USBCDCMSD usb(&msd_handler, 0xffff, 0x32fc, 0);

#ifdef DEBUG
IOStream stream(uart1);
#endif

//const AP_HAL::HAL& hal = AP_HAL_XPCC;
//extern const AP_HAL::HAL& hal;

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

class DFU final : public DFUHandler {
public:
	void do_detach() {
		dfu_detach = true;
	}
};
DFU dfu;

Radio radio;

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

using namespace xpcc;
using namespace stm32;


void SD_LowLevel_init() {
	PC8::setFunction(AltFunction::AF_SDIO);
	PC8::setPullMode(GPIOPuPd::GPIO_PuPd_UP);
	PC8::setSpeed(GPIOSpeed::GPIO_Speed_50MHz);

	PC9::setFunction(AltFunction::AF_SDIO);
	PC9::setPullMode(GPIOPuPd::GPIO_PuPd_UP);
	PC9::setSpeed(GPIOSpeed::GPIO_Speed_50MHz);

	PC10::setFunction(AltFunction::AF_SDIO);
	PC10::setPullMode(GPIOPuPd::GPIO_PuPd_UP);
	PC10::setSpeed(GPIOSpeed::GPIO_Speed_50MHz);

	PC11::setFunction(AltFunction::AF_SDIO);
	PC11::setPullMode(GPIOPuPd::GPIO_PuPd_UP);
	PC11::setSpeed(GPIOSpeed::GPIO_Speed_50MHz);

	PC12::setFunction(AltFunction::AF_SDIO);
	PC12::setPullMode(GPIOPuPd::GPIO_PuPd_NOPULL);
	PC12::setSpeed(GPIOSpeed::GPIO_Speed_50MHz);

	PD2::setFunction(AltFunction::AF_SDIO);
	PD2::setPullMode(GPIOPuPd::GPIO_PuPd_UP);
	PD2::setSpeed(GPIOSpeed::GPIO_Speed_50MHz);
}

void UART_LowLevel_init() {

	U1Rx::setPullMode(GPIOPuPd::GPIO_PuPd_UP);
	U2Rx::setPullMode(GPIOPuPd::GPIO_PuPd_UP);
	U6Rx::setPullMode(GPIOPuPd::GPIO_PuPd_UP);

	U1Tx::setFunction(AltFunction::AF_USART1);
	U1Rx::setFunction(AltFunction::AF_USART1);

	U2Tx::setFunction(AltFunction::AF_USART2);
	U2Rx::setFunction(AltFunction::AF_USART2);

	U6Tx::setFunction(AltFunction::AF_USART6);
	U6Rx::setFunction(AltFunction::AF_USART6);
}

void SPI_LowLevel_init() {
	nRadioSel::setOutput(1);

	RadioMiso::setFunction(AltFunction::AF_SPI1);
	RadioMiso::setSpeed(GPIOSpeed::GPIO_Speed_50MHz);

	RadioMosi::setFunction(AltFunction::AF_SPI1);
	RadioMosi::setSpeed(GPIOSpeed::GPIO_Speed_50MHz);

	RadioSck::setFunction(AltFunction::AF_SPI1);
	RadioSck::setSpeed(GPIOSpeed::GPIO_Speed_50MHz);

	RadioIrq::setPullMode(GPIOPuPd::GPIO_PuPd_UP);

	radioSpiMaster::initialize(radioSpiMaster::Prescaler::Div8,
			radioSpiMaster::MasterSelection::Master,
			radioSpiMaster::DataMode::Mode0);

}

void I2C_LowLevel_init() {
	//Scl::setOutputType(GPIOOType::GPIO_OType_OPENDRAIN);

	//Drive SCL clock. This increases robustness of i2c comms.
	//Note that clock stretching devices are not supported in this case.
	Scl::setOutputType(GPIOOType::GPIO_OType_PUSHPULL);
	Scl::setSpeed(GPIOSpeed::GPIO_Speed_2MHz);
	Scl::setPullMode(GPIOPuPd::GPIO_PuPd_UP);


	Sda::setOutputType(GPIOOType::GPIO_OType_OPENDRAIN);
	Sda::setSpeed(GPIOSpeed::GPIO_Speed_2MHz);
	Sda::setPullMode(GPIOPuPd::GPIO_PuPd_UP);

	Scl::setFunction(AltFunction::AF_I2C1);
	Sda::setFunction(AltFunction::AF_I2C1);

	stm32::I2cMaster1::initialize();
}

void dbgset(uint8_t i) {
	PB15::set();
}

void dbgclr(uint8_t i) {
	PB15::reset();
}
void dbgtgl(uint8_t i) {
	PB15::toggle();
}

//#define DEBUG

#ifdef DEBUG
template <typename Task, size_t stack_size>
class ChTask : public Task, chibios_rt::BaseStaticThread<stack_size> {
public:
	template<typename ... Args>
	ChTask(Args ... args) :
			Task(args...) {
	}

protected:
	void handleInit() {
		chibios_rt::BaseStaticThread<stack_size>::start(NORMALPRIO);
		this->Task::handleInit();
	}

	void handleTick() override {
		chThdYield();
	}

	void _yield(uint16_t timeAvailable) override {
		chThdYield();
	}

	void main() override {
		while(1) {
			this->Task::handleTick();
			_yield(0);
		}
	}

};



class T2: TickerTask {

	void handleInit() {
		//hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&T2::test, void));
	}

	void test() {
		uint8_t data2[14];
		uint8_t addr = 0x3B;

		xpcc::I2cWriteReadTransaction t;
		t.initialize(0x68, &addr, 1, data2, 14);

		xpcc::stm32::I2cMaster1::start(&t);

		t.wait();

		if(t.getState() == I2cWriteReadTransaction::AdapterState::Idle) {
//					XPCC_LOG_DEBUG .dump_buffer(data2, 14);
//
//					XPCC_LOG_DEBUG .printf("A %d %d %d\n",
//							(int16_t)__builtin_bswap16(*(int16_t*)&data2[0]),
//							(int16_t)__builtin_bswap16(*(int16_t*)&data2[2]),
//							(int16_t)__builtin_bswap16(*(int16_t*)&data2[4]));

			//XPCC_LOG_DEBUG .printf("G %d %d %d\n");
		} else {
			XPCC_LOG_DEBUG .printf("error\n");
		}
	}

	void handleTick() {
		int c;

		static uint8_t b[512] __aligned(4);

		if ((c = uart1.read()) >= 0) {
			if (c == 'r') {
				NVIC_SystemReset();
			}

			if (c == 'y') {
				xpcc::I2cWriteTransaction adapter;

				uint8_t buf[3] = { 0x0F };

				XPCC_LOG_DEBUG << "Scanning i2c bus\n";
				for (int i = 0; i < 128; i++) {
					adapter.initialize(i, buf, 1);
					if (!I2cMaster1::start(&adapter)) {
						XPCC_LOG_DEBUG.printf("start failed\n");
					}
					while (adapter.isBusy())
						;

					if (I2cMaster1::getErrorState()
							!= xpcc::I2cMaster::Error::AddressNack) {
						XPCC_LOG_DEBUG.printf("Found device @ 0x%x %d %d\n", i,
								I2cMaster1::getErrorState(),
								adapter.getState());
					}
				}
			}

			if(c == '4') {
				radio.reset();
			}

			if (c == 't') {
				I2cWriteReadTransaction adapter;
				uint8_t data1 = 0;
				uint8_t c = 0x75;

				uint8_t data2 = 0;
				if (hal.i2c->readRegister(0x68, 0x0, &data1) == 1) {
					XPCC_LOG_DEBUG.printf("failed\n");
				}

				XPCC_LOG_DEBUG.printf("%x\n", data1);

			}

			if (c == 'x') {
				uint8_t data2[12];
				//while(1) {
				if (hal.i2c->readRegisters(0x68, 0x0C, 2, data2) == 1) {
					XPCC_LOG_DEBUG.printf("failed\n");
				}
				//}
				XPCC_LOG_DEBUG.printf("%x\n", data2[0]);
			}

			if (c == 'z') {
				uint8_t data2[12];
				//while(1) {
				if (hal.i2c->readRegisters(0x68, 0x0C, 4, data2) == 1) {
					XPCC_LOG_DEBUG.printf("failed\n");
				}
				//}
				XPCC_LOG_DEBUG.printf("%x\n", data2[0]);
			}

			if (c == 's') {
				memset(b, 0, 512);

//			uint32_t c = stm32::GPTimer5::getValue();
//			for (int i = 0; i < 10; i++) {
//				sd.readSingleBlock(b, i);
//			}
//			c = stm32::GPTimer5::getValue() - c;
//			XPCC_LOG_DEBUG.printf("time %d\n", c);

				//uint32_t c = stm32::GPTimer5::getValue();
				//for(int i = 0; i < (1024*1024)/512; i++) {
				//	sd.readSingleBlock(b, i);
				//}
				//c = stm32::GPTimer5::getValue() - c;
				//XPCC_LOG_DEBUG .printf("time %d\n", c );

				//sdCard.doRead(b, 1, 2);
				sdCard.doRead(b, 0, 1);
				XPCC_LOG_DEBUG.dump_buffer(b, 512);

			}

			if (c == 'b') {
				__disable_irq();
				 *((unsigned long *)0x2000FFF0) = 0xDEADBEEF;
				NVIC_SystemReset();

			}

			if (c == '2') {

				sdCard.writeStart(1, 5);


				memset((uint8_t*) b, 0x11, 512);
				sdCard.writeData(b);

				memset((uint8_t*) b, 0x22, 512);
				sdCard.writeData(b);

				memset((uint8_t*) b, 0x33, 512);
				sdCard.writeData(b);

				memset((uint8_t*) b, 0x44, 512);
				sdCard.writeData(b);

				memset((uint8_t*) b, 0x55, 512);
				sdCard.writeData(b);

				sdCard.writeStop();

				sdCard.writeStart(1, 1);
				memset((uint8_t*) b, 0x11, 512);
				sdCard.writeData(b);
				sdCard.writeStop();

			}

			if (c == '3') {

				memset((uint8_t*) b, 0x11, 512);

				for(int i = 5; i < 15; i++) {
					sdCard.writeBlock(i, b);
				}

			}

			if (c == 'm') {
				sdCard.readStart(1);

				uint32_t c = stm32::GPTimer5::getValue();
				for (int i = 0; i < 100; i++) {
					sdCard.readData(b, i);
				}
				c = stm32::GPTimer5::getValue() - c;
				XPCC_LOG_DEBUG.printf("time %d\n", c);

				sdCard.readData(b, 512);

				XPCC_LOG_DEBUG.dump_buffer(b, 512);

				sdCard.readData(b, 512);
				XPCC_LOG_DEBUG.dump_buffer(b, 512);

				sdCard.readData(b, 512);
				XPCC_LOG_DEBUG.dump_buffer(b, 512);

				sdCard.readStop();

			}

			if (c == 'i') {
				EXTI->SWIER |= (1 << 0);
			}

			if(c == '[') {
				uint8_t buf[255];
				memset(buf, 0, 255);

				hal.storage->read_block(&buf, 2000, 16);
				XPCC_LOG_DEBUG .dump_buffer(buf, 16);
			}

			if(c == 'k') {
				while(1) {
					XPCC_LOG_DEBUG << 'a';
				}

			}

			if(c == '>') {
				((XpccHAL::Storage*)hal.storage)->block_cache.dump();
			}
			if(c == '*') {
				volatile float testas = 0.45f;
				testas /= 0.0f;
			}

			if(c == ']') {
				static uint8_t g = 0;
				g++;
				uint8_t buf[255];
				for(int i =0; i < 255; i++) {
					buf[i] = g;
				}
				XPCC_LOG_DEBUG .dump_buffer(buf, 16);
				hal.storage->write_block(2000, buf, 16);
			}

			if(c == '0') {
				uint8_t data2[14];
				uint8_t addr = 0x3B;

				xpcc::I2cWriteReadTransaction t;
				t.initialize(0x68, &addr, 1, data2, 14);

				xpcc::stm32::I2cMaster1::start(&t);

				t.wait();

				if(t.getState() == I2cWriteReadTransaction::AdapterState::Idle) {
					XPCC_LOG_DEBUG .dump_buffer(data2, 14);

					XPCC_LOG_DEBUG .printf("A %d %d %d\n",
							(int16_t)__builtin_bswap16(*(int16_t*)&data2[0]),
							(int16_t)__builtin_bswap16(*(int16_t*)&data2[2]),
							(int16_t)__builtin_bswap16(*(int16_t*)&data2[4]));

					XPCC_LOG_DEBUG .printf("G %d %d %d\n",
							(int16_t)__builtin_bswap16(*(int16_t*)&data2[8]),
							(int16_t)__builtin_bswap16(*(int16_t*)&data2[10]),
							(int16_t)__builtin_bswap16(*(int16_t*)&data2[12]));

					//XPCC_LOG_DEBUG .printf("G %d %d %d\n");
				} else {
					XPCC_LOG_DEBUG .printf("error\n");
				}

			}

		}
		chThdYield();
	}


};

T2 test;
#endif

class USBStorage : chibios_rt::BaseStaticThread<512> {
public:
	USBStorage() {
		msd_handler.set_device_strings("SkyVideo", "SKY.Falcon", "1.0");

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
};

USBStorage usb_storage_task;


//extern AP_HAL::HAL::Callbacks copter;
void HAL_XPCC::run(int argc, char * const argv[], Callbacks* callbacks) const {
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
	PB15::setOutput(0);
	PB13::setOutput(0);

	XPCC_LOG_DEBUG << "Init Board\n";
	XPCC_LOG_DEBUG .printf("fAPB1 %d\n", Clocks::getPCLK1Frequency());
	XPCC_LOG_DEBUG .printf("fAPB2 %d\n", Clocks::getPCLK2Frequency());
	XPCC_LOG_DEBUG .printf("fAHB %d\n", Clocks::getHCLKFrequency());

	sdCard.init();

    hal.scheduler->init(0);
    hal.analogin->init(0);
    hal.rcout->init(0);
    hal.rcin->init(0);
    hal.i2c->begin();
    hal.storage->init(0);

	NVIC_SetPriority(USART1_IRQn, 4);
	NVIC_SetPriority(USART2_IRQn, 4);
	NVIC_SetPriority(USART6_IRQn, 4);

	I2cMaster1::setIrqPriority(5);

	NVIC_SetPriority(OTG_FS_IRQn, 8);
	NVIC_SetPriority(SDIO_IRQn, 8);

	radio.init();

	usb.connect();

	//NVIC_EnableIRQ(FPU_IRQn);
	//NVIC_SetPriority(FPU_IRQn, 0);
	//setup();
	callbacks->setup();

#ifndef DEBUG
	IWDG->KR = 0x5555;
	IWDG->PR = 0x3;
	IWDG->KR = 0x5555;
	IWDG->RLR = 0xFFF; //4096ms timeout

	IWDG->KR = 0xCCCC; //start the watchdog
#endif

	for(;;) {
		//dbgset();
		callbacks->loop();
		//dbgtgl();
		//dbgclr();
		sdCard.update();
//		static PeriodicTimer<> t(1000);
//		if(t.isExpired()) {
//			printf("%d\n", usb.suspended());
//		}

		if(dfu_detach) {
			sleep(100);
			hal.scheduler->reboot(true);
		}
	}
}

