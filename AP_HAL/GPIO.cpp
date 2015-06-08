
#include "GPIO.h"

using namespace XpccHAL;

#include <xpcc/architecture.hpp>
#include <AP_HAL.h>
#include <pindefs.hpp>

extern const AP_HAL::HAL& hal;

GPIO::GPIO()
{}

void GPIO::init()
{}

void GPIO::pinMode(uint8_t pin, uint8_t output)
{
	//special pins
	if((pin>>4) == 0x0F) {
		return;
	}


}

int8_t GPIO::analogPinToDigitalPin(uint8_t pin)
{
	switch(pin) {
	case 10:
		return xpcc::stm32::PC0::Id;
	case 11:
		return xpcc::stm32::PC1::Id;
	case 12:
		return xpcc::stm32::PC2::Id;
	case 13:
		return xpcc::stm32::PC3::Id;
	case 14:
		return xpcc::stm32::PC4::Id;
	case 15:
		return xpcc::stm32::PC5::Id;
	}

	return -1;
}


uint8_t GPIO::read(uint8_t pin) {
	//handle LEDs
	if(pin == 0xFF) {

	}
	if(pin == 0xFE) {

	}
	if(pin == 0xFD) {

	}
	/////
}

void GPIO::write(uint8_t pin, uint8_t value)
{
	//handle LEDs
	if(pin == 0xFF) {

	}
	if(pin == 0xFE) {

	}
	if(pin == 0xFD) {

	}
	/////

}

void GPIO::toggle(uint8_t pin)
{
	write(pin, !read(pin));
}

/* Alternative interface: */
AP_HAL::DigitalSource* GPIO::channel(uint16_t n) {
    return new DigitalSource(n);
}

/* Interrupt interface: */
bool GPIO::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
        uint8_t mode) {

	uint8_t port = PORT(interrupt_num);
	uint8_t pin = PIN(interrupt_num);

	xpcc::IntEdge m;
    if (mode == HAL_GPIO_INTERRUPT_FALLING)
    	m = xpcc::IntEdge::FALLING_EDGE;
    else if (mode == HAL_GPIO_INTERRUPT_RISING)
    	m = xpcc::IntEdge::RISING_EDGE;
    else {
    	return false;
    }

	if(port == 2 || port == 0) {
		xpcc::GpioInt::attach(port, pin, p, m);
		return true;
	}

    return false;
}

DigitalSource::DigitalSource(uint8_t pin) :
    _pin(pin)
{}

void DigitalSource::mode(uint8_t output)
{
	hal.gpio->pinMode(_pin, output);
}

uint8_t DigitalSource::read() {
    return hal.gpio->read(_pin);
}

void DigitalSource::write(uint8_t value) {
	hal.gpio->write(_pin, value);
}

void DigitalSource::toggle() {
	hal.gpio->toggle(_pin);
}
