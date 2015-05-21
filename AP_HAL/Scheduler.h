
#ifndef __AP_HAL_EMPTY_SCHEDULER_H__
#define __AP_HAL_EMPTY_SCHEDULER_H__

#include "AP_HAL_XPCC.h"

#define MAX_TIMER_PROCS 4

class XpccHAL::Scheduler final : public AP_HAL::Scheduler {
public:
    Scheduler();
    void     init(void* machtnichts);
    void     delay(uint16_t ms);
    uint32_t millis();
    uint32_t micros();
    //uint64_t millis64();
   // uint64_t micros64();
    void     delay_microseconds(uint16_t us);
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

    void     panic(const prog_char_t *errormsg);
    void     reboot(bool hold_in_bootloader);

    void	 yield();

private:
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

    static void _timer_procs_timer_event();
};

#endif // __AP_HAL_EMPTY_SCHEDULER_H__
