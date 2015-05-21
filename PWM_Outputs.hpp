/*
 * PWM_Outputs.hpp
 *
 *  Created on: May 11, 2015
 *      Author: walmis
 */

#ifndef PWM_OUTPUTS_HPP_
#define PWM_OUTPUTS_HPP_

#include <xpcc/architecture.hpp>

using namespace xpcc;
using namespace stm32;
/*
 *  1 - T1_CH1
 *  2 - T1_CH2
 *  3 - T1_CH3
 *  4 - T2_CH2
 *  5 - T2_CH1
 *  6 - T3_CH1
 *  7 - T3_CH2
 *  8 - T4_CH1
 *  AUX_9 - T2_CH3
 */

class PWM_Outputs {
public:
	void init() {
		PA0::setFunction(AltFunction::AF_TIM2);
		PA1::setFunction(AltFunction::AF_TIM2);
		PA8::setFunction(AltFunction::AF_TIM1);
		PB0::setFunction(AltFunction::AF_TIM1);
		PB1::setFunction(AltFunction::AF_TIM1);
		PB4::setFunction(AltFunction::AF_TIM3);
		PB5::setFunction(AltFunction::AF_TIM3);
		PB6::setFunction(AltFunction::AF_TIM4);

		stm32::GPTimer2::enable();
		stm32::GPTimer3::enable();
		stm32::GPTimer4::enable();
		stm32::AdvTimer1::enable();

		setFrequency(490);

		stm32::AdvTimer1::configureOutputChannel(1,
					stm32::AdvTimer1::OutputCompareMode::Pwm, 1000);
		stm32::AdvTimer1::configureOutputChannel(2,
					stm32::AdvTimer1::OutputCompareMode::Pwm, 1000);
		stm32::AdvTimer1::configureOutputChannel(3,
					stm32::AdvTimer1::OutputCompareMode::Pwm, 1000);

		stm32::AdvTimer1::configureOutputChannel(1,
				stm32::AdvTimer1::OutputCompareMode::Pwm,
				stm32::AdvTimer1::PinState::Enable,
				stm32::AdvTimer1::OutputComparePolarity::ActiveHigh,
				stm32::AdvTimer1::PinState::Disable);

		stm32::AdvTimer1::configureOutputChannel(2,
				stm32::AdvTimer1::OutputCompareMode::Pwm,
				stm32::AdvTimer1::PinState::Disable,
				stm32::AdvTimer1::OutputComparePolarity::ActiveHigh,
				stm32::AdvTimer1::PinState::Enable);

		stm32::AdvTimer1::configureOutputChannel(3,
				stm32::AdvTimer1::OutputCompareMode::Pwm,
				stm32::AdvTimer1::PinState::Disable,
				stm32::AdvTimer1::OutputComparePolarity::ActiveHigh,
				stm32::AdvTimer1::PinState::Enable);

		stm32::GPTimer2::configureOutputChannel(1,
					stm32::GPTimer2::OutputCompareMode::Pwm, 1000);
		stm32::GPTimer2::configureOutputChannel(2,
					stm32::GPTimer2::OutputCompareMode::Pwm, 1000);
		stm32::GPTimer3::configureOutputChannel(1,
					stm32::GPTimer3::OutputCompareMode::Pwm, 1000);
		stm32::GPTimer3::configureOutputChannel(2,
					stm32::GPTimer3::OutputCompareMode::Pwm, 1000);
		stm32::GPTimer4::configureOutputChannel(1,
					stm32::GPTimer4::OutputCompareMode::Pwm, 1000);

		stm32::AdvTimer1::enableOutput();
		stm32::AdvTimer1::start();
		stm32::GPTimer2::start();
		stm32::GPTimer3::start();
		stm32::GPTimer4::start();
	}

	void setOutput(uint8_t channel, uint16_t pulse) {

		switch(channel) {
		case 0:
			stm32::AdvTimer1::setCompareValue(1, pulse);
			break;
		case 1:
			stm32::AdvTimer1::setCompareValue(2, pulse);
			break;
		case 2:
			stm32::AdvTimer1::setCompareValue(3, pulse);
			break;
		case 3:
			stm32::GPTimer2::setCompareValue(2, pulse);
			break;
		case 4:
			stm32::GPTimer2::setCompareValue(1, pulse);
			break;
		case 5:
			stm32::GPTimer3::setCompareValue(1, pulse);
			break;
		case 6:
			stm32::GPTimer3::setCompareValue(2, pulse);
			break;
		case 7:
			stm32::GPTimer4::setCompareValue(1, pulse);
			break;
		case 8:
			break;
		}
	}

	void setFrequency(uint32_t f) {
		period_us = 1000000/f;

		stm32::AdvTimer1::setPrescaler(96);
		stm32::AdvTimer1::setOverflow(period_us);
		stm32::GPTimer2::setPrescaler(48);
		stm32::GPTimer2::setOverflow(period_us);
		stm32::GPTimer3::setPrescaler(48);
		stm32::GPTimer3::setOverflow(period_us);
		stm32::GPTimer4::setPrescaler(48);
		stm32::GPTimer4::setOverflow(period_us);

	}

	uint16_t period_us;

};

#endif /* PWM_OUTPUTS_HPP_ */
