/*
 * lowlevel.hpp
 *
 *  Created on: Aug 24, 2016
 *      Author: walmis
 */

#ifndef LOWLEVEL_HPP_
#define LOWLEVEL_HPP_


	//SDIO pins
void 	SD_LowLevel_init();
	////
void 	UART_LowLevel_init();
	///
void 	SPI_LowLevel_init();
	///
void 	I2C_LowLevel_init();

void 	wdt_init();
#endif /* LOWLEVEL_HPP_ */
