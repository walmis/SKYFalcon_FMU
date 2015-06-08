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

#include <xpcc/architecture.hpp>
#include <xpcc/processing.hpp>
#include <xpcc/debug.hpp>
#include <math.h>

#define USB_PRODUCT_STRING		"SKY.Falcon FMU"
#define USB_MANUFACTURER_STRING	"SKYVideo.pro"
#define USB_SERIAL_STRING		"0001"

#define CDC_EPBULK_IN	EP2IN
#define CDC_EPBULK_OUT	EP2OUT
#define CDC_EPINT_IN	9

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

BufferedUart<stm32::Usart2> uart2(57600, 128, 128);
BufferedUart<stm32::Usart1> uart1(230400, 512, 128);
BufferedUart<stm32::Usart6> uartGps(57600, 128, 128);

xpcc::log::Logger xpcc::log::debug(uart1);
xpcc::log::Logger xpcc::log::error(uart1);
xpcc::log::Logger xpcc::log::info(uart1);
xpcc::log::Logger xpcc::log::warning(uart1);


SDCardVolume<stm32::SDIO_SDCard> sdCard;

fat::FileSystem fs(&sdCard);
USBMSD_VolumeHandler msd_handler(&sdCard, 2048);

//USBSerial usb;
USBCDCMSD usb(&msd_handler, 0xffff, 0x32fc, 0);


IOStream stream(uart1);

const AP_HAL::HAL& hal = AP_HAL_XPCC;
//extern const AP_HAL::HAL& hal;

void delay(uint32_t ms)
{
    hal.scheduler->delay(ms);
}

void mavlink_delay(uint32_t ms)
{
    hal.scheduler->delay(ms);
}

uint32_t millis()
{
    return hal.scheduler->millis();
}

uint32_t micros()
{
    return hal.scheduler->micros();
}

XpccHAL::UARTDriver uartADriver(&usb.serial);
XpccHAL::UARTDriver uartBDriver(0);
XpccHAL::UARTDriver uartCDriver(0);
XpccHAL::UARTDriver uartDDriver(0);
XpccHAL::UARTDriver uartEDriver(0);
XpccHAL::UARTDriver uartConsoleDriver(&uart1);



class Vol : public USBMSDHandler {

    void transfer_begins(TransferType type, uint32_t startBlock, int numBlocks) {
    	//XPCC_LOG_DEBUG .printf("tr start total %d\n", numBlocks);

    }
    int disk_read_start(uint8_t * data, uint32_t block, uint32_t blocksLeft) {
    	memset(data, 0xcc, 512);
    	disk_read_finalize(true);
    	return DISK_OK;
    }
    int disk_write_start(const uint8_t * data, uint32_t block, uint32_t blocksLeft) {
    	disk_write_finalize(true);
    	return DISK_OK;
    }

    int disk_initialize() {
    	return DISK_OK;
    }
    uint32_t disk_sectors() {
    	return 1024*1024;
    }
    uint16_t disk_sector_size() {
    	return 512;
    }
    int disk_status() {
    	return DISK_OK;
    }

};



//IOStream usbserial(usb);
//USBCDCMSD<USBMSD_VolumeHandler> usb(0xFFFF, 0x5678, 0, &sdCard);


Radio radio;

bool storage_lock;

void XpccHAL::UARTDriver::setBaud(uint32_t baud, xpcc::IODevice* device) {
//	if(device == &uartGps) {
//		uartGps.setBaud(baud);
//	}
}

bool XpccHAL::GPIO::usb_connected(void)
{
	return false;
	//return !usb.suspended();
}

extern "C"
void _delay_ms(uint32_t ms) {
	Timeout<> t(ms);
	while(!t.isExpired());
}

using namespace xpcc;
using namespace stm32;


void boot_flag() {
	RTC->BKP0R |= (1<<31);
	NVIC_SystemReset();
}



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
	Scl::setOutputType(GPIOOType::GPIO_OType_OPENDRAIN);
	Scl::setSpeed(GPIOSpeed::GPIO_Speed_50MHz);
	Scl::setPullMode(GPIOPuPd::GPIO_PuPd_UP);


	Sda::setOutputType(GPIOOType::GPIO_OType_OPENDRAIN);
	Sda::setSpeed(GPIOSpeed::GPIO_Speed_50MHz);
	Sda::setPullMode(GPIOPuPd::GPIO_PuPd_UP);

	Scl::setOutput(1);

	for(int i = 0; i < 2*9; i++) {
		Scl::reset();
		sleep(1);
		Scl::set();
		sleep(1);
	}

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

//
//class ChTask_Base {
//
//public:
//	ChTask_Base(void* stack, size_t stacksize);
//
//
//protected:
//	virtual void run() = 0;
//	void _yield(uint16_t timeAvailable);
//
//	void _handleTick();
//
//	void _thread() {
//		while(1) {
//			run();
//		}
//	}
//
//	void* sp;
//	void* stack;
//	uint16_t stacksize;
//	xpcc::Timestamp sleep_timeout;
//	uint8_t flags;
//};

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

class MyTask : public TickerTask {
protected:
	void handleTick() {
			XPCC_LOG_DEBUG .printf("Task\n");
			chThdSleepMilliseconds(333);
	}
};

//ChTask<MyTask, 512> TestCoop;


class Test: TickerTask {

	void handleTick() {
		static PeriodicTimer<> t(500);

		if (t.isExpired()) {

				//XPCC_LOG_DEBUG .printf("Board %.3f\n", hal.analogin->board_voltage());
				//XPCC_LOG_DEBUG.printf("I2C CR1=%x CR2=%x SR1=%x\n", I2C1->CR1,
				//I2C1->CR2, I2C1->SR1);

				//PB15::reset();

				//usb.write('A');
				//usbserial << "Hello world\n";

				//XPCC_LOG_DEBUG.printf("PSP %x\n", __get_PSP());

				LedRed::toggle();
				//LedGreen::toggle();
				//LedBlue::toggle();
				//PB15::set();

				//XPCC_LOG_DEBUG .printf("ADC1->SR 0x%x %x %x\n", ADC1->SR, DMA2_Stream0->NDTR, DMA2_Stream0->CR);
				//XPCC_LOG_DEBUG .printf("s[0] %d\n",reinterpret_cast<XpccHAL::AnalogIn*>(hal.analogin)->samples[0]);
				//XPCC_LOG_DEBUG .printf("s[1] %d\n",reinterpret_cast<XpccHAL::AnalogIn*>(hal.analogin)->samples[1]);
				//XPCC_LOG_DEBUG .printf("s[2] %d\n",reinterpret_cast<XpccHAL::AnalogIn*>(hal.analogin)->samples[2]);
				//XPCC_LOG_DEBUG .printf("s[3] %d\n",reinterpret_cast<XpccHAL::AnalogIn*>(hal.analogin)->samples[3]);
			//chThdYield();
		}

		//static uint8_t buf[64];
		//memset(buf, 'A', 64);
		//usb.writeNB(CDC_EPBULK_IN, buf, 64, 64);
	}

};

Test task;


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
				if (hal.i2c->readRegisters(0x68, 0x0C, 1, data2) == 1) {
					XPCC_LOG_DEBUG.printf("failed\n");
				}
				//}
				XPCC_LOG_DEBUG.printf("%x\n", data2[0]);
			}

			if (c == 's') {
				static __aligned(4) uint8_t b[1024];
				memset(b, 0, 1024);

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
				sdCard.doRead(b, 0, 2);
				XPCC_LOG_DEBUG.dump_buffer(b, 1024);

				sdCard.doRead(b, 2, 2);
				XPCC_LOG_DEBUG.dump_buffer(b, 1024);
			}

			if (c == 'b') {

				//sd.writeStart(1, 0);

				uint8_t b[512];
				memset((uint8_t*) b, 0xBA, 512);
				sdCard.writeBlock(1, b);
				//
				//				memset((uint8_t*)b, 0xCC, 512);
				//				sd.writeBlock(2, b);
				//
				//				memset((uint8_t*)c, 0x55, 512);
				//				sd.writeBlock(3, b);
				//
				//				memset((uint8_t*)c, 0xDD, 512);
				//				sd.writeBlock(4, b);
				//
				//				memset((uint8_t*)c, 0xEE, 512);
				//				sd.writeBlock(5, b);

				//sd.writeStop();

				//sd.writeBlock(1, (uint8_t*)b);
			}

			if (c == '2') {

				sdCard.writeStart(1, 5);

				uint8_t b[512];
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

				uint8_t b[512];
				sdCard.writeStart(1, 10);

				for(int i = 0; i < 10000; i++) {

					memset((uint8_t*) b, (uint8_t)i, 512);

					if(!sdCard.writeData(b)) {
						XPCC_LOG_DEBUG .printf("block %d failed\n", i+1);
					}

				}

				sdCard.writeStop();

			}

			if (c == 'm') {
				static uint8_t b[512];
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


static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");

  while (true) {
    LedBlue::toggle();
    chThdSleepMilliseconds(200);
    LedBlue::toggle();
    chThdSleepMilliseconds(200);

  }
}


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
			LedGreen::reset();
			msd_handler.waitForEvent();
			LedGreen::set();
			msd_handler.run();
		}
	}
};

USBStorage usb_storage_task;

extern void setup();
extern void loop();



int main() {
	stm32::SysTickTimer::enable();
	//boot_flag();
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

	sdCard.init();

	//port_timer_start_alarm(1000000);

	XPCC_LOG_DEBUG .printf("f APB1 %d\n", Clocks::getPCLK1Frequency());
	XPCC_LOG_DEBUG .printf("f APB2 %d\n", Clocks::getPCLK2Frequency());
	XPCC_LOG_DEBUG .printf("f AHB %d\n", Clocks::getHCLKFrequency());

	hal.init(0, 0);

	XPCC_LOG_DEBUG .printf("ipsr %d %x\n", __get_IPSR(), new uint32_t[4]);

	chThdCreateStatic(waThread1, sizeof(waThread1), HIGHPRIO, Thread1, NULL);

	//chThdYield();

	//XPCC_LOG_DEBUG .printf("main stack size %d\n", &__main_stack_end__ - &__main_stack_base__ );

	usb.connect();

	NVIC_SetPriority(USART1_IRQn, 4);
	NVIC_SetPriority(USART2_IRQn, 4);
	NVIC_SetPriority(USART6_IRQn, 4);

	NVIC_SetPriority(I2C1_EV_IRQn, 5);
	NVIC_SetPriority(I2C1_ER_IRQn, 5);

	NVIC_SetPriority(OTG_FS_IRQn, 8);
	NVIC_SetPriority(SDIO_IRQn, 8);

	radio.init();

	TickerTask::tasksInit();
	//TickerTask::tasksRun();

	setup();

	for(;;) {
		dbgset();
		loop();
		dbgclr();
		TickerTask::tick();
	}

}
