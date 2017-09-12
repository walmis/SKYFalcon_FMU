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

void Scheduler::init()
{

	auto it = MpuInt::attachInterrupt([this]() {
		//mpu6k_evt.signal();
		chibios_rt::System::lockFromIsr();
		mpu_evt.broadcastFlagsI(MPU_EVENT_MASK);
		chibios_rt::System::unlockFromIsr();

	}, xpcc::IntEdge::RISING_EDGE);
	it.leak();

	Thread::start(HIGHPRIO-1);

	_timer_proc_enabled = true;

	xpcc::stm32::SysTickTimer::attachInterrupt(Scheduler::_failsafe_timer_event);
}

void Scheduler::delay(uint16_t ms)
{
	if(ms == 0) return;

	uint32_t start = AP_HAL::micros();
    while (ms > 0) {
    	uint32_t nextTimeout = AP_HAL::micros() + 1000;

        if (_min_delay_cb_ms <= ms) {
            if (_delay_proc) {
            	_delay_proc();
            }
        }
        if(AP_HAL::micros() < nextTimeout) {
        	chibios_rt::BaseThread::sleepUntil(nextTimeout);
        }
        ms--;
    }
}



void Scheduler::_failsafe_timer_event() {
	if(_failsafe) {
		_failsafe();
	}
}


void Scheduler::delay_microseconds_boost(uint16_t us) {
	if(us == 0) return;
	//chibios_rt::BaseThread::setPriority(HIGHPRIO);
	//ch.mainthread.p_prio = HIGHPRIO;
	chibios_rt::BaseThread::sleep(us);

	//chVTSet(&timer, 150, restore_priority, (void*)0);
	//chibios_rt::BaseThread::setPriority(NORMALPRIO);
}

chibios_rt::EvtSource* Scheduler::getSync() {
	return &mpu_evt;
}

void Scheduler::main() {
	chibios_rt::BaseThread::setName("HALTimer");

	while(1) {
		uint32_t nextDeadline = chibios_rt::System::getTimeX() + MS2ST(1);

		IWDG->KR = 0xAAAA; //feed the watchdog

		_run_timer_procs(true);

		if(chibios_rt::System::getTimeX() < nextDeadline)
			chThdSleepUntil(nextDeadline);
	}
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
    	AP_HAL::panic("Failed to register timer proc\n");
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



void Scheduler::reboot(bool hold_in_bootloader) {
	if(hold_in_bootloader) {
		*(uint32_t*)0x2000FFF0 = 0xDEADBEEF;
	}

	NVIC_SystemReset();
}

void Scheduler::yield() {
	chThdYield();
}
