#include <xpcc/architecture.hpp>
#include "Scheduler.h"
#include "AnalogIn.h"
#include "../pindefs.hpp"

using namespace XpccHAL;

extern const AP_HAL::HAL& hal;
extern volatile uint32_t timerOverflowCount;

AP_HAL::Proc Scheduler::_failsafe = 0;
volatile bool      Scheduler::_timer_proc_enabled = false;
volatile bool      Scheduler::_timer_event_missed = false;
volatile bool      Scheduler::_in_timer_proc = false;
AP_HAL::MemberProc Scheduler::_timer_proc[MAX_TIMER_PROCS] = {0};
volatile uint8_t   Scheduler::_num_timer_procs = 0;

Scheduler::Scheduler()
{}

void Scheduler::init(void* machtnichts)
{
	MpuInt::attachInterrupt([this]() {
		mpu6k_evt.signal();
	}, xpcc::IntEdge::RISING_EDGE);

	Thread::start(HIGHPRIO-1);

	_timer_proc_enabled = true;

	xpcc::stm32::SysTickTimer::attachInterrupt(Scheduler::_failsafe_timer_event);
}

void Scheduler::delay(uint16_t ms)
{
	if(ms == 0) return;

	uint32_t start = micros();
    while (ms > 0) {
    	uint32_t nextTimeout = micros() + 1000;

        if (_min_delay_cb_ms <= ms) {
            if (_delay_proc) {
            	_delay_proc();
            }
        }
        if(micros() < nextTimeout) {
        	chibios_rt::BaseThread::sleepUntil(nextTimeout);
        }
        ms--;
    }
}

extern volatile bool dfu_detach;

void Scheduler::_failsafe_timer_event() {
	if(_failsafe) {
		_failsafe();
	}
}

static void restore_priority(void*) {
	chibios_rt::System::lockFromIsr();
	ch.mainthread.p_prio = NORMALPRIO;
	//ch.mainthread.p_realprio = NORMALPRIO;
	chibios_rt::System::unlockFromIsr();
}

void Scheduler::delay_microseconds_boost(uint16_t us) {
	if(us == 0) return;

	static virtual_timer_t timer;

	//chibios_rt::BaseThread::setPriority(HIGHPRIO);
	//ch.mainthread.p_prio = HIGHPRIO;
	chibios_rt::BaseThread::sleep(us);

	//chVTSet(&timer, 150, restore_priority, (void*)0);
	//chibios_rt::BaseThread::setPriority(NORMALPRIO);
}


void Scheduler::main() {
	chibios_rt::BaseThread::setName("HALTimer");

	while(1) {
		//uint32_t nextDeadline = chibios_rt::System::getTimeX() + MS2ST(1);
		IWDG->KR = 0xAAAA; //feed the watchdog
		//xpcc::stm32::PB13::set();
		_run_timer_procs(true);
		//xpcc::stm32::PB13::reset();
		//synchronize on MPU6050 1khz DRDY
		mpu6k_evt.wait(1);
	}
}

uint32_t Scheduler::millis() {
    return xpcc::Clock::now().getTime();
}

uint32_t Scheduler::micros() {
    return chibios_rt::System::getTimeX();
}

uint64_t Scheduler::millis64() {
    return xpcc::Clock::now().getTime();
}

uint64_t Scheduler::micros64() {
	__disable_irq();
	static uint32_t last_time;
	static uint32_t thigh = 0;
	uint32_t time = chibios_rt::System::getTimeX();
	//handle timer overflow
	if(time < last_time) {
		thigh++;
	}
	last_time = time;

    uint64_t ret = ((uint64_t)thigh)<<32 | time ;
    __enable_irq();
    return ret;
}

void Scheduler::delay_microseconds(uint16_t us)
{
	if(us==0) return;
	chibios_rt::BaseThread::sleep(us);
}

void Scheduler::register_delay_callback(AP_HAL::Proc k,
            uint16_t min_time_ms)
{
	_delay_proc = k;
	_min_delay_cb_ms = min_time_ms;
}

void Scheduler::register_timer_process(AP_HAL::MemberProc proc)
{
	for (int i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i] == proc) {
            return;
        }
    }

    if (_num_timer_procs < MAX_TIMER_PROCS) {
        /* this write to _timer_proc can be outside the critical section
         * because that memory won't be used until _num_timer_procs is
         * incremented. */
        _timer_proc[_num_timer_procs] = proc;
        /* _num_timer_procs is used from interrupt, and multiple bytes long. */
        _num_timer_procs++;
    } else {
    	panic("Failed to register timer proc\n");
    }
}

void Scheduler::register_io_process(AP_HAL::MemberProc k)
{
}

void Scheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us)
{
	_failsafe = failsafe;
}

void Scheduler::suspend_timer_procs()
{
	_timer_proc_enabled = false;
}

void Scheduler::resume_timer_procs()
{
	_timer_proc_enabled = true;
    if (_timer_event_missed == true) {
    	_run_timer_procs(false);
        _timer_event_missed = false;
    }
}

void Scheduler::_run_timer_procs(bool called_from_isr)
{
    if (_in_timer_proc) {
        return;
    }

	_in_timer_proc = true;

    static uint8_t last_proc_id = 0;

    if (_timer_proc_enabled) {
        // now call the timer based drivers
        for (int i = last_proc_id; i < _num_timer_procs; i++) {
            if (_timer_proc[i]) {
            	_timer_proc[i]();
            }
        }
//        for (int i = 0; i < last_proc_id; i++) {
//            if (_timer_proc[i] != NULL) {
//                _timer_proc[i]();
//            }
//        }

        //last_proc_id++;
        //if(last_proc_id >= _num_timer_procs) last_proc_id = 0;

    } else if (called_from_isr) {
        _timer_event_missed = true;
    }

    _in_timer_proc = false;

}

bool Scheduler::in_timerprocess() {
    return _in_timer_proc;
}

void Scheduler::begin_atomic()
{
	__disable_irq();
}

void Scheduler::end_atomic()
{
	__enable_irq();
}

bool Scheduler::system_initializing() {
    return false;
}

void Scheduler::system_initialized()
{

}

void Scheduler::panic(const prog_char_t *errormsg) {
	XPCC_LOG_ERROR << errormsg << xpcc::endl;
    hal.console->println_P(errormsg);
    LedBlue::set();
    LedRed::set();
    LedGreen::set();
    for(;;) {

		if(dfu_detach) {
			sleep(100);
			hal.scheduler->reboot(true);
		}

    }
}

void Scheduler::reboot(bool hold_in_bootloader) {
	if(hold_in_bootloader) {
		*(uint32_t*)0x2000FFF0 = 0xDEADBEEF;
	}

	NVIC_SystemReset();
}

void Scheduler::yield() {
	chThdYield();
}
