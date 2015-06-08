/*
 * pindefs.hpp
 *
 *  Created on: May 21, 2015
 *      Author: walmis
 */

#ifndef PINDEFS_HPP_
#define PINDEFS_HPP_

#include <xpcc/architecture.hpp>

typedef xpcc::stm32::PC13 LedRed;
typedef xpcc::stm32::PC14 LedBlue;
typedef xpcc::stm32::PC15 LedGreen;

typedef xpcc::stm32::PA4 nRadioSel;
typedef xpcc::stm32::PA5 RadioSck;
typedef xpcc::stm32::PA6 RadioMiso;
typedef xpcc::stm32::PA7 RadioMosi;
typedef xpcc::stm32::PB3 RadioIrq;

typedef xpcc::stm32::PA9  U1Tx;
typedef xpcc::stm32::PA10 U1Rx;

typedef xpcc::stm32::PA2 U2Tx;
typedef xpcc::stm32::PA3 U2Rx;

typedef xpcc::stm32::PC6 U6Tx;
typedef xpcc::stm32::PC7 U6Rx;

typedef xpcc::stm32::PB8 Scl;
typedef xpcc::stm32::PB9 Sda;

typedef xpcc::stm32::PB7 MpuInt;

typedef xpcc::stm32::PB10 ExtGpioTim2Ch3;
typedef xpcc::stm32::PB12 ExtGpioSel;
typedef xpcc::stm32::PB13 ExtGpioSck;
typedef xpcc::stm32::PB14 ExtGpioMosi;
typedef xpcc::stm32::PB15 ExtGpioMiso;
typedef xpcc::stm32::PC0 ExtGpioAdc0;
typedef xpcc::stm32::PC1 ExtGpioAdc1;
typedef xpcc::stm32::PC2 ExtGpioAdc2;
typedef xpcc::stm32::PC3 ExtGpioAdc3;
typedef xpcc::stm32::PC4 ExtVoltage;
typedef xpcc::stm32::PC5 ExtCurrent;

typedef xpcc::stm32::Spi1 radioSpiMaster;

#endif /* PINDEFS_HPP_ */
