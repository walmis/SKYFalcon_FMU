/*
 * radio.cpp
 *
 *  Created on: Sep 24, 2014
 *      Author: walmis
 */

#include "radio.hpp"
#include "pindefs.hpp"

using namespace xpcc;
using namespace xpcc::stm32;

extern const AP_HAL::HAL& hal;

void Radio::_irq_entry(void *ths) {
	static_cast<Radio*>(ths)->irqTask();
}

void Radio::_main_entry(void *ths) {
	static_cast<Radio*>(ths)->mainTask();
}

void Radio::irqTask() {
	chibios_rt::BaseThread::setName("RadioIRQ");
	while(1) {
		irqEvent.wait(10);
		if(!RadioIrq::read()) {
			this->RH_RF22::handleInterrupt();
		}
//		if(irqEvent.wait(10)) {
//			this->RH_RF22::handleInterrupt();
//		} else {
//			if(!RadioIrq::read()) {
//				printf("BUG: event timeout but interrupt is set\n");
//				this->RH_RF22::handleInterrupt();
//			}
//		}
	}
}

void Radio::mainTask() {
	chibios_rt::BaseThread::setName("Radio");

	while(1) {
		dataEvent.wait(10);

		if (!transmitting()) {
			if (rxDataLen) {

				noiseFloor = ((uint16_t) noiseFloor * 31 + rssiRead()) / 32;

				uint8_t* buf = rxBuf;
				uint8_t len = rxDataLen;

				if (len >= sizeof(Packet)) {
					Packet* inPkt = (Packet*) buf;

					switch (inPkt->id) {
					case PACKET_RC:
						if(len >= sizeof(RCPacket)) {
							rcData = *((RCPacket*) inPkt);
							rcPacketTimestamp = Clock::now();

							uint8_t payload_len = len - sizeof(RCPacket);
							if(payload_len) {
								if(inPkt->ackSeq == seq) {
									rxbuf.write(buf+sizeof(RCPacket), payload_len);
								} else {
									XPCC_LOG_DEBUG
									.printf("discard duplicate seq:%d ackSeq:%d\n", seq, inPkt->ackSeq);

								}
							}
						}

						break;
					case PACKET_RF_PARAM_SET: {
						RadioCfgPacket* cfg = (RadioCfgPacket*) inPkt;

						printf("--Radio parameters received--\n");
						printf("Freq %d\n", cfg->frequency/1000);
						printf("AfcPullIn %d\n", cfg->afcPullIn/1000);
						printf("Modem setting %d\n", cfg->modemCfg);
						printf("FH Channels %d\n", cfg->fhChannels);
						printf("TX Power %d\n", cfg->txPower);
						setModeIdle();
						if (radio_cfg.frequency != cfg->frequency/1000) {
							setFrequency(cfg->frequency/1000, cfg->afcPullIn/1000);
						}

						if (cfg->modemCfg != radio_cfg.modemCfg.get()) {
							setModemConfig((RH_RF22::ModemConfigChoice) cfg->modemCfg);
						}

						if (radio_cfg.txPower.get() != cfg->txPower) {
							setTxPower(cfg->txPower);
						}

						radio_cfg.fhChannels.set_and_save_ifchanged(cfg->fhChannels);
						setModeRx();

					}
						break;
					}

					//received packet, send data back
					//if no data is available, send only ack
					if (inPkt->id >= PACKET_RC) {
						//lastAckSeq = inPkt->ackSeq; //set last acknowledged ours packet
						if (radio_cfg.fhChannels)
							setFHChannel((inPkt->seq ^ 0x55) % radio_cfg.fhChannels);

						//dbgclr();
						//printf("*\n");
						sendAck(inPkt);

					}
				}

				rxDataLen = 0;
			}
			//XPCC_LOG_DEBUG .dump_buffer(buf, len);
		}
	}
}

bool Radio::init() {
	if(!hwInitialized) {
#if 1
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
#endif
		if(!RH_RF22::HWinit()) {
			hal.scheduler->panic("Radio HW init failed");
			return false;
		}
	}
	hwInitialized = true;

	if(!RH_RF22::init()) {
		hal.scheduler->panic("Radio init failed");
	}

	radio_cfg.frequency.load();
	radio_cfg.modemCfg.load();
	radio_cfg.txPower.load();
	radio_cfg.fhChannels.load();

	setFHStepSize(10);
	setFrequency(radio_cfg.frequency.get());
	setTxPower(radio_cfg.txPower.get());
	setModemConfig((RH_RF22::ModemConfigChoice)radio_cfg.modemCfg.get());

	setModeRx();
	XPCC_LOG_DEBUG .printf("Radio initialized (f:%d)\n", radio_cfg.frequency.get());

	thread_t* tirq = chThdCreateStatic(_irq_wa, sizeof(_irq_wa), NORMALPRIO+3, _irq_entry, this);
	thread_t* tmain = chThdCreateStatic(_main_wa, sizeof(_main_wa), NORMALPRIO, _main_entry, this);


	return true;
}


bool Radio::sendAck(Packet* inPkt) {
	Packet* out = (Packet*)txBuf;

	if((uint16_t)radio_cfg.maxFragment > (RH_RF22_MAX_MESSAGE_LEN - sizeof(Packet))) {
		radio_cfg.maxFragment = RH_RF22_MAX_MESSAGE_LEN - sizeof(Packet);
	} else if((uint16_t)radio_cfg.maxFragment < 32) {
		radio_cfg.maxFragment = 32;
	}
	uint16_t maxFrag = radio_cfg.maxFragment;

	out->noise = getNoiseFloor();
	out->rssi = getRssi();
	//if last sent packet was a data packet, retry transmission
	if(inPkt->ackSeq != seq && out->id == PACKET_DATA) {
		//retry last data transmission
		printf("Retry id:%d len:%d\n", out->id, dataLen);
		out->seq = ++seq;
		out->ackSeq = inPkt->seq;

		RH_RF22::send(txBuf, dataLen);
	} else {
		uint16_t txavail = txbuf.bytes_used();
		out->seq = ++seq;
		out->ackSeq = inPkt->seq; //acknowledge received packet
		out->id = PACKET_DATA;

		if(txavail) {
			uint8_t* buf = txBuf + sizeof(Packet);

			for(int i = 0; i < std::min(maxFrag, txavail); i++) {
				*buf++ = popTx();
			}

			dataLen = buf - txBuf;
			//printf("Send data %d\n", dataLen);
			RH_RF22::send(txBuf, dataLen);

		} else {
			out->id = PACKET_ACK; //no data to send, send ack
			RH_RF22::send(txBuf, sizeof(Packet));
		}
	}

	return true;
}

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

void Radio::handleRxComplete() {
	rssi = (rssi * 7 + (uint8_t)lastRssi()) / 8;

	if(available()) {
		if(rxDataLen) {
			XPCC_LOG_DEBUG .printf("packet not cleared\n");
		}
		rxDataLen = sizeof(rxBuf);
		if(!recv(rxBuf, (uint8_t*)&rxDataLen)) {
			rxDataLen = 0;
		}
		dataEvent.signal();
	}
}

void Radio::handleInterrupt() {
	irqEvent.signal();
}

void Radio::handleTxComplete() {
	setModeRx();
	dataEvent.signal();
}

void Radio::handleReset() {
	XPCC_LOG_DEBUG .printf("RF22 Reset!\n");
}
