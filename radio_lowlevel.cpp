/*
 * radio_lowlevel.cpp
 *
 *  Created on: Sep 5, 2017
 *      Author: walmis
 */

#include "radio.hpp"
#include "pindefs.hpp"


uint8_t Radio::spiBurstWrite(uint8_t reg, const uint8_t* src, uint8_t len) {

    uint8_t status = 0;
    ATOMIC_BLOCK_START;

	//XPCC_LOG_DEBUG .printf("dmaw\n");
	dmatx.disable();
	dma_txEvt.reset();

	dmatx.setCurrDataCounter(len);
	dmatx.memoryTargetConfig((uint32_t)src, dma::Memory::Memory_0);
	dmatx.setMemoryInc(dma::MemoryInc::Enable);

    digitalWrite(_slaveSelectPin, LOW);
    status = _spi.transfer(reg | RH_SPI_WRITE_MASK); // Send the start address with the write mask on

	dmatx.enable();

	if(!dma_txEvt.wait(10)) {
		XPCC_LOG_DEBUG << "SPI DMA TX Evt timeout\n";
	}

	dmatx.disable();
	//empty the rx buffer
	SPI1->DR;

    digitalWrite(_slaveSelectPin, HIGH);
    ATOMIC_BLOCK_END;
    return status;
}

uint8_t Radio::spiBurstRead(uint8_t reg, uint8_t* dest, uint8_t len) {
	static uint16_t dummy = 0;

    uint8_t status = 0;
    ATOMIC_BLOCK_START;

	dma_rxEvt.reset();
	dmarx.disable();
	dmatx.disable();

	dmarx.setCurrDataCounter(len);
	dmarx.memoryTargetConfig((uint32_t)dest, dma::Memory::Memory_0);
	dmatx.setCurrDataCounter(len);
	dmatx.memoryTargetConfig((uint32_t)&dummy, dma::Memory::Memory_0);
	dmatx.setMemoryInc(dma::MemoryInc::Disable);

	dmarx.setXferDirection(dma::XferDir::PeripheralToMemory);

    digitalWrite(_slaveSelectPin, LOW);
    status = _spi.transfer(reg & ~RH_SPI_WRITE_MASK); // Send the start address with the write mask off

	dmarx.enable();
	dmatx.enable();

	if(!dma_rxEvt.wait(10)) {
		XPCC_LOG_DEBUG << "SPI DMA RX Evt timeout\n";
	}

	dmarx.disable();
	dmatx.disable();

    digitalWrite(_slaveSelectPin, HIGH);
    ATOMIC_BLOCK_END;
    return status;

}

void Radio::initDMA() {
	dma::Config cfg;
	cfg.periphBaseAddress((uint32_t)&SPI1->DR)
			->memoryDataSize(dma::MemoryDataSize::Byte)
			->peripheralDataSize(dma::PeripheralDataSize::Byte)
			->xferDirection(dma::XferDir::PeripheralToMemory)
			->channel(dma::Channel::Channel_3)
			->memoryInc(dma::MemoryInc::Enable);

	dmarx.init(cfg);
	dmarx.attachCallback([this]() {
		//XPCC_LOG_DEBUG .printf("dmarc\n");
		dma_rxEvt.signal();
	});

	cfg.xferDirection(dma::XferDir::MemoryToPeripheral)
					   ->channel(dma::Channel::Channel_3);

	dmatx.init(cfg);
	dmatx.attachCallback([this]() {
		//XPCC_LOG_DEBUG .printf("dmatc\n");
		dma_txEvt.signal();
	});

	SPI1->CR2 |= SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;
}
