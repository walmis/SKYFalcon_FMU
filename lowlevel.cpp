/*
 * lowlevel.cpp
 *
 *  Created on: Aug 24, 2016
 *      Author: walmis
 */

#include "pindefs.hpp"

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

void wdt_init() {
#ifndef DEBUG
	IWDG->KR = 0x5555;
	IWDG->PR = 0x3;
	IWDG->KR = 0x5555;
	IWDG->RLR = 0xFFF; //4096ms timeout

	IWDG->KR = 0xCCCC; //start the watchdog
#endif
}

extern "C"
float __ieee754_sqrtf(float op1) {
    	float result;
	asm volatile ("vsqrt.f32 %0, %1" : "=w" (result) : "w" (op1) );
	return (result);
}
