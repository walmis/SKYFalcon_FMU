
#include <xpcc/architecture.hpp>

#include <wirish.h>
#include <RHGenericSPI.h>
#include <stdarg.h>


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

void rh_atomic_block_start() {
//	xpcc::GpioInt::disableInterrupts();
}

void rh_atomic_block_end() {
//	xpcc::GpioInt::enableInterrupts();
}

//uint32_t millis() {
//	return xpcc::Clock::now().getTime();
//}

void pinMode(uint8_t pin, WiringPinMode mode) {

//	LPC_GPIO_TypeDef* g = 0;
	uint8_t p = pin & 0x1F;
	switch(pin>>5) {
	case 0:
//		g = LPC_GPIO0;
		break;
	case 1:
//		g = LPC_GPIO1;
		break;
	case 2:
//		g = LPC_GPIO2;
		break;
	default:
		return;
	}

	switch(mode) {
	case WiringPinMode::OUTPUT:
//		g->FIODIR |= (1<<p);
		break;
	case WiringPinMode::INPUT:
//		g->FIODIR &= ~(1<<p);
		break;
	default:
		return;
	}
}

void attachInterrupt(uint8_t pin, void (*fn)(void), int mode) {

//	xpcc::GpioInt::attach(pin>>5, pin&0x1F,
//			fn, xpcc::IntEdge::FALLING_EDGE);

}


void digitalWrite(uint8_t pin, uint8_t val) {
//	uint8_t p = pin&0x1F;
//	switch(pin>>5) {
//	case 0:
//		val ? LPC_GPIO0->FIOSET|=(1<<p):LPC_GPIO0->FIOCLR|=(1<<p);
//		break;
//	case 1:
//		val ? LPC_GPIO1->FIOSET|=(1<<p):LPC_GPIO1->FIOCLR|=(1<<p);
//		break;
//	case 2:
//		val ? LPC_GPIO2->FIOSET|=(1<<p):LPC_GPIO2->FIOCLR|=(1<<p);
//		break;
//	}
}

uint8_t digitalRead(uint8_t pin) {
//	uint8_t p = pin&0x1F;
//	switch(pin>>5) {
//	case 0:
//		return (LPC_GPIO0->FIOPIN & (1<<p)) != 0;
//	case 1:
//		return (LPC_GPIO1->FIOPIN & (1<<p)) != 0;
//	case 2:
//		return (LPC_GPIO2->FIOPIN & (1<<p)) != 0;
//	}
	return 0;
}

class Spi : RHGenericSPI {

public:
	void begin() {}
	void end() {}

	uint8_t transfer(uint8_t data) {
		//printf("spi write %02x\n", data);
//		return radioSpiMaster::write(data);
	}
};

Spi hardware_spi;
