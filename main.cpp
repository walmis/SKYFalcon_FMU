/*
 * main.cpp
 *
 *  Created on: Apr 7, 2015
 *      Author: walmis
 */

#include <xpcc/architecture.hpp>
#include <xpcc/processing.hpp>
#include <math.h>

#include <xpcc/driver/connectivity/usb/USBDevice/USBDevice/USBHAL.h>
#include <xpcc/architecture/peripheral/i2c_adapter.hpp>

using namespace xpcc;
using namespace stm32;

extern "C"
void HardFault_Handler() {
	//GPIOD->MODER = (1<<(12*2));
	//GPIOD->ODR = (1<<12);

	while(1) {
		__asm__("BKPT");
	}
}

USBHAL hal;

class Test {
public:
	Test() {
		PD15::setOutput(1);
	}
};
Test test;

dma::DMAStream st(dma::Stream::DMA2_0);

uint8_t buf0[128];
uint8_t buf1[128];

int main() {
	SystemCoreClockUpdate();
	memset(buf0, 0xCC, 128);

	stm32::SysTickTimer::enable();

	PA0::setFunction(AltFunction::AF_TIM2);
	PA1::setFunction(AltFunction::AF_TIM2);
	PA6::setFunction(AltFunction::AF_TIM3);

	stm32::GPTimer3::enable();

	uint32_t ovf = stm32::GPTimer3::setPeriod(2000);

	stm32::GPTimer3::configureOutputChannel(1,
			stm32::GPTimer3::OutputCompareMode::Pwm, ovf/3);

	stm32::GPTimer3::configureOutputChannel(2,
			stm32::GPTimer3::OutputCompareMode::Pwm, ovf/4);


	stm32::GPTimer3::start();


	stm32::I2cMaster1::initialize();

	dma::Config cfg;
	cfg.bufferSize(128)
			->xferDirection(dma::XferDir::MemoryToMemory)
			->memoryInc(dma::MemoryInc::Enable)
			->peripheralInc(dma::PeripheralInc::Enable)
			->periphBaseAddress((uint32_t)buf0)
			->memory0BaseAddress((uint32_t)buf1);


	st.init(cfg);
	st.enable();


	PB6::setFunction(AltFunction::AF_I2C1);
	PB9::setFunction(AltFunction::AF_I2C1);
	//PB6::setOutput();
	//PA9::setFunction(GPIO_AF_USART1);

	PD12::setOutput();
	PD13::setOutput();
	PD14::setOutput();

	stm32::Usart1::initialize(57600);

	PeriodicTimer<> t(500);

	hal.connect();

	xpcc::I2cWriteReadAdapter delegate;

	uint8_t buf[2] = {0x55, 0x55};

	volatile uint32_t count = 0;
	while(1) {
		if(t.isExpired()) {
			PD12::toggle();
			PB6::toggle();
			PD13::toggle();
			//PD14::toggle();

			//stm32::Usart1::put('A');

			delegate.initialize(0x55, buf,2, 0, 0);
			stm32::I2cMaster1::start(&delegate);

		}

		count++;
	}
}
