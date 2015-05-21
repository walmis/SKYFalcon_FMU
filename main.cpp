/*
 * main.cpp
 *
 *  Created on: Apr 7, 2015
 *      Author: walmis
 */

#include <xpcc/architecture.hpp>
#include <xpcc/processing.hpp>
#include <xpcc/debug.hpp>
#include <math.h>

#include <xpcc/driver/connectivity/usb/USBDevice/USBSerial/USBSerial.h>
#include <xpcc/architecture/peripheral/i2c_adapter.hpp>

#include "PWM_Outputs.hpp"
#include "AP_HAL/UARTDriver.h"
#include "AP_HAL/GPIO.h"
#include "AP_HAL/AnalogIn.h"

USBSerial usb;

BufferedUart<stm32::Usart2> uart(57600, 128, 128);
xpcc::log::Logger xpcc::log::debug(uart);
xpcc::log::Logger xpcc::log::error(uart);

IOStream stream(uart);
PWM_Outputs pwm;

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

XpccHAL::UARTDriver uartADriver(0);
XpccHAL::UARTDriver uartBDriver(0);
XpccHAL::UARTDriver uartCDriver(0);
XpccHAL::UARTDriver uartDDriver(0);
XpccHAL::UARTDriver uartEDriver(0);
XpccHAL::UARTDriver uartConsoleDriver(0);

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


uint8_t buf0[128];
uint8_t buf1[128];

void boot_flag() {
	RTC->BKP0R |= (1<<31);
	NVIC_SystemReset();
}

stm32::SDIO_SDCard sd;

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

	PA3::setPullMode(GPIOPuPd::GPIO_PuPd_UP);

	PA2::setFunction(AltFunction::AF_USART2);
	PA3::setFunction(AltFunction::AF_USART2);
}


class Test : TickerTask {

	void handleTick() {
		static PeriodicTimer<> t(500);

		if (t.isExpired()) {
			PC13::toggle();

			XPCC_LOG_DEBUG .printf("Board %.3f\n", hal.analogin->board_voltage());


			//XPCC_LOG_DEBUG .printf("ADC1->SR 0x%x %x %x\n", ADC1->SR, DMA2_Stream0->NDTR, DMA2_Stream0->CR);
			//XPCC_LOG_DEBUG .printf("s[0] %d\n",reinterpret_cast<XpccHAL::AnalogIn*>(hal.analogin)->samples[0]);
			//XPCC_LOG_DEBUG .printf("s[1] %d\n",reinterpret_cast<XpccHAL::AnalogIn*>(hal.analogin)->samples[1]);
			//XPCC_LOG_DEBUG .printf("s[2] %d\n",reinterpret_cast<XpccHAL::AnalogIn*>(hal.analogin)->samples[2]);
			//XPCC_LOG_DEBUG .printf("s[3] %d\n",reinterpret_cast<XpccHAL::AnalogIn*>(hal.analogin)->samples[3]);
		}
	}
};

Test task;


class T2 : TickerTask {
	void handleTick() {
	int c;
	if ((c = uart.read()) >= 0) {
		if (c == 'r') {
			NVIC_SystemReset();
		}


		if (c == 's') {
			uint8_t b[512];
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

			sd.readSingleBlock(b, 1);
			XPCC_LOG_DEBUG.dump_buffer(b, 512);

			sd.readSingleBlock(b, 2);
			XPCC_LOG_DEBUG.dump_buffer(b, 512);

			sd.readSingleBlock(b, 3);
			XPCC_LOG_DEBUG.dump_buffer(b, 512);

			sd.readSingleBlock(b, 4);
			XPCC_LOG_DEBUG.dump_buffer(b, 512);

			sd.readSingleBlock(b, 5);
			XPCC_LOG_DEBUG.dump_buffer(b, 512);
		}

		if (c == 'b') {

			//sd.writeStart(1, 0);

			uint8_t b[512];
			memset((uint8_t*) b, 0xBA, 512);
			sd.writeBlock(1, b);
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

			sd.writeStart(1, 5);

			uint8_t b[512];
			memset((uint8_t*) b, 0x11, 512);
			sd.writeData(b);

			memset((uint8_t*) b, 0x22, 512);
			sd.writeData(b);

			memset((uint8_t*) b, 0x33, 512);
			sd.writeData(b);

			memset((uint8_t*) b, 0x44, 512);
			sd.writeData(b);

			delay_ms(50);

			memset((uint8_t*) b, 0x55, 512);
			sd.writeData(b);

			sd.writeStop();

		}

		if (c == 'm') {
			static uint8_t b[512];
			sd.readStart(1);

			uint32_t c = stm32::GPTimer5::getValue();
			for (int i = 0; i < 100; i++) {
				sd.readData(b, i);
			}
			c = stm32::GPTimer5::getValue() - c;
			XPCC_LOG_DEBUG.printf("time %d\n", c);

			sd.readData(b, 512);

			XPCC_LOG_DEBUG.dump_buffer(b, 512);

			sd.readData(b, 512);
			XPCC_LOG_DEBUG.dump_buffer(b, 512);

			sd.readData(b, 512);
			XPCC_LOG_DEBUG.dump_buffer(b, 512);

			sd.readStop();

		}

		if(c == 'i') {
			EXTI->SWIER |= (1<<0);
		}
	}
}
};

T2 test;


void intr() {
	PC14::toggle();
}

class Testas {

public:
void test() {
  static volatile int i = 0;
  i++;
  //std::cout << "hello\n";
}
};

Testas testas;


int main() {
	memset(buf0, 0xCC, 128);

	stm32::SysTickTimer::enable();
	//boot_flag();
	//SDIO pins
	SD_LowLevel_init();
	////
	UART_LowLevel_init();

	pwm.init();

	pwm.setOutput(3, 1500);
	pwm.setOutput(4, 1500);
	pwm.setOutput(7, 1500);
	pwm.setOutput(2, 1500);


	stm32::I2cMaster1::initialize();


	PC13::setOutput();
	PC14::setOutput();

	PeriodicTimer<> t(500);

	usb.connect();

	xpcc::I2cWriteReadAdapter delegate;

	uint8_t buf[2] = {0x55, 0x55};

	sd.init();

	XPCC_LOG_DEBUG .printf("f APB1 %d\n", Clocks::getPCLK1Frequency());
	XPCC_LOG_DEBUG .printf("f APB2 %d\n", Clocks::getPCLK2Frequency());
	XPCC_LOG_DEBUG .printf("f AHB %d\n", Clocks::getHCLKFrequency());

	hal.init(0, 0);

	PC4::setAnalog();
	PC5::setAnalog();

	AP_HAL::AnalogSource* ch = hal.analogin->channel(14);
	AP_HAL::AnalogSource* ch2 = hal.analogin->channel(15);



	XPCC_LOG_DEBUG .printf("ipsr %d %x\n", __get_IPSR(), new uint32_t[4]);

	TickerTask::tasksRun();

}
