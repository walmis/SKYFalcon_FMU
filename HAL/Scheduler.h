
#ifndef __AP_HAL_EMPTY_SCHEDULER_H__
#define __AP_HAL_EMPTY_SCHEDULER_H__

#include "../HAL/AP_HAL_XPCC.h"
#include <ch.hpp>

#define MAX_TIMER_PROCS 4

class XpccHAL::Scheduler final : public AP_HAL::Scheduler,
	chibios_rt::BaseStaticThread<768> {

	typedef chibios_rt::BaseStaticThread<768> Thread;
public:
    Scheduler();
    void     init();
    void     delay(uint16_t ms);

    void     delay_microseconds(uint16_t us);
    //void	 delay_microseconds_boost(uint16_t us);
    void     register_delay_callback(AP_HAL::Proc,
                uint16_t min_time_ms);

    void     register_timer_process(AP_HAL::MemberProc);
    void     register_io_process(AP_HAL::MemberProc);
    void     suspend_timer_procs();
    void     resume_timer_procs();

    bool     in_timerprocess();

    void     register_timer_failsafe(AP_HAL::Proc, uint32_t period_us);

    void     begin_atomic();
    void     end_atomic();

    bool     system_initializing();
    void     system_initialized();

    void     reboot(bool hold_in_bootloader);

    void	 yield();

    static constexpr eventmask_t MPU_EVENT_MASK = EVENT_MASK(31);

    //return 1khz sync from MPU6000
    chibios_rt::EvtSource* getSync();

private:
    //xpcc::Event mpu6k_evt;
    chibios_rt::EvtSource mpu_evt;

    //timer thread
    void main();

    AP_HAL::Proc volatile _delay_proc;
    volatile uint16_t _min_delay_cb_ms;

    static void _run_timer_procs(bool called_from_isr);

    static volatile bool _timer_proc_enabled;
    static volatile bool _timer_event_missed;

    static volatile bool _in_timer_proc;

    static void _failsafe_timer_event();
    static AP_HAL::Proc _failsafe;

    static AP_HAL::MemberProc _timer_proc[MAX_TIMER_PROCS];
    static volatile uint8_t _num_timer_procs;

};

#endif // __AP_HAL_EMPTY_SCHEDULER_H__
