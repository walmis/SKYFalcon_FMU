
#include <xpcc/architecture.hpp>
#include <wirish.h>
#include <RHGenericSPI.h>
#include <stdarg.h>
#include "pindefs.hpp"

using namespace xpcc::stm32;

int printf(const char* fmt, ...) {
	va_list ap;
	va_start(ap, fmt);

	XPCC_LOG_DEBUG .vprintf(fmt, ap);

	va_end(ap);

	return 0;
}

int puts(const char* s) __attribute__((used));
int puts(const char* s) {
	XPCC_LOG_DEBUG << s << xpcc::endl;
	return 1;
}

#ifdef putchar
#undef putchar
#endif

int putchar(int c) {
	XPCC_LOG_DEBUG << c;
	return 1;
}

//void delay(uint32_t millis) {
//	xpcc::delay_ms(millis);
//}

void rh_yield() {
	//xpcc::TickerTask::yield();
}

chibios_rt::Mutex mtx;

void rh_atomic_block_start() {
	mtx.lock();
}

void rh_atomic_block_end() {
	mtx.unlock();
}

//uint32_t millis() {
//	return xpcc::Clock::now().getTime();
//}

void pinMode(uint8_t pin, WiringPinMode mode) {

	switch(mode) {
	case WiringPinMode::OUTPUT:
		_GpioPin::setOutput(GPIO_IDToPort(pin), GPIO_IDToPin(pin));
		break;
	case WiringPinMode::INPUT:
		_GpioPin::setInput(GPIO_IDToPort(pin), GPIO_IDToPin(pin));
		break;
	default:
		return;
	}
}

void attachInterrupt(uint8_t pin, void (*fn)(void), int mode) {
	_GpioPin::attachInterrupt(GPIO_IDToPort(pin), GPIO_IDToPin(pin), fn, xpcc::IntEdge::FALLING_EDGE);
}

void digitalWrite(uint8_t pin, uint8_t val) {
	_GpioPin::set(GPIO_IDToPort(pin), GPIO_IDToPin(pin), val);
}

uint8_t digitalRead(uint8_t pin) {
	_GpioPin::read(GPIO_IDToPort(pin), GPIO_IDToPin(pin));
	return 0;
}

class Spi : RHGenericSPI {

public:
	void begin() {}
	void end() {}

	uint8_t transfer(uint8_t data) {
		radioSpiMaster::write(data);
		while(!radioSpiMaster::isReceiveRegisterNotEmpty());
		uint16_t ret;
		radioSpiMaster::read(ret);
		return ret;
	}
};

Spi hardware_spi;
