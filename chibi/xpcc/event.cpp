#include <xpcc/processing/rtos_abstraction.hpp>
#include <ch.h>
#include <chbsem.h>

namespace xpcc {

/*
 * Event handling structure
 *
 * Thread calls Wait to wait for some event occured in interrupt.
 * Eg. character received from UART
 * The IRQ handler then calls event.signal() to notify the listener
 *
 */

// class Event  {
// public:
	bool Event::isPending() {
    binary_semaphore_t* bsem = (binary_semaphore_t*)event;
		return chBSemWaitTimeout(bsem, TIME_IMMEDIATE) == MSG_TIMEOUT;
	}

	void Event::reset() {
		(bool)isPending();
	}

	bool Event::wait(uint32_t timeout_ms) {
		if(TickerTask::inInterruptContext()) return false;
    binary_semaphore_t* bsem = (binary_semaphore_t*)event;

		//(bool)isPending(); //this resets the event into waiting state

		if(timeout_ms == 0xFFFFFFFF) {
			return chBSemWait(bsem) == MSG_OK;
		} else {
			return chBSemWaitTimeout(bsem, MS2ST(timeout_ms)) == MSG_OK;
		}
	}

	bool Event::wait_us(uint32_t timeout_us) {
		if(TickerTask::inInterruptContext()) return false;
    binary_semaphore_t* bsem = (binary_semaphore_t*)event;

		if(timeout_us == 0xFFFFFFFF) {
			return chBSemWait(bsem) == MSG_OK;
		} else {
			return chBSemWaitTimeout(bsem, US2ST(timeout_us)) == MSG_OK;
		}
	}

	//signal waiting threads
	void Event::signal() {
		syssts_t s = chSysGetStatusAndLockX();
		chBSemSignalI((binary_semaphore_t*)event);
		chSysRestoreStatusX(s);
	}

  Event::Event() {
    binary_semaphore_t* bsem = new binary_semaphore_t;
    chBSemObjectInit(bsem, true);
    event = (void*)bsem;
  }

  Event::~Event() {
    binary_semaphore_t* bsem = (binary_semaphore_t*)event;
    delete bsem;
  }

}

// protected:
// 	chibios_rt::BinarySemaphore cond{true};
// };

// }
