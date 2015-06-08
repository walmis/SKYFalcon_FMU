/*
 * chbAlarm.cpp
 *
 *  Created on: May 27, 2015
 *      Author: walmis
 */

#include "ch.h"
#include <xpcc/architecture.hpp>

using namespace xpcc::stm32;

extern "C"
void port_timer_init() {
//	//TIM5 counts microseconds
	xpcc::stm32::GPTimer5::enable();
	xpcc::stm32::GPTimer5::setPrescaler(SystemCoreClock / 2000000);
	xpcc::stm32::GPTimer5::applyAndReset();
	xpcc::stm32::GPTimer5::start();
//	//////

	NVIC_SetPriority(TIM5_IRQn, 3);
	NVIC_EnableIRQ(TIM5_IRQn);
}

extern "C"
void TIM5_IRQHandler() {
	CH_IRQ_PROLOGUE();

	/* Note, under rare circumstances an interrupt can remain latched even if
	 the timer SR register has been cleared, in those cases the interrupt
	 is simply ignored.*/
	if ((TIM5->SR & TIM_SR_CC1IF) != 0U) {
		chSysLockFromISR();

		TIM5->SR = 0U;
		chSysTimerHandlerI();

		chSysUnlockFromISR();
	}


    //xpcc::stm32::PB15::toggle();
	//XPCC_LOG_DEBUG .printf("tim5\n");

	CH_IRQ_EPILOGUE();
}

extern "C"
void port_timer_start_alarm(systime_t time) {
	//XPCC_LOG_DEBUG .printf("tm %d\n", time);
	TIM5->CCR1 = time;
	TIM5->SR = 0;
	TIM5->DIER = TIM_DIER_CC1IE;
}

extern "C"
void port_timer_stop_alarm(void) {
	TIM5->DIER = 0;
}

/**
 * @brief   Sets the alarm time.
 *
 * @param[in] time      the time to be set for the next alarm
 *
 * @notapi
 */
extern "C"
void port_timer_set_alarm(systime_t time) {
	TIM5->CCR1 = time;
}

/**
 * @brief   Returns the system time.
 *
 * @return              The system time.
 *
 * @notapi
 */
extern "C"
systime_t port_timer_get_time(void) {
	return GPTimer5::getValue();
}
/**
 * @brief   Returns the current alarm time.
 *
 * @return              The currently set alarm time.
 *
 * @notapi
 */
extern "C"
systime_t port_timer_get_alarm(void) {
	return GPTimer5::getCompareValue(1);
}
