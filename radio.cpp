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
    chRegSetThreadName("RadioIRQ");
    
	while(1) {
		chEvtWaitAnyTimeout(EVENT_IRQ, MS2ST(10));
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
    chRegSetThreadName("Radio");
	static xpcc::PeriodicTimer<> checkTimer(100);

	if(!initRadioRegisters()) {
		AP_HAL::panic("Radio init failed");
	}

	while(1) {
		eventmask_t events = chEvtWaitAnyTimeout(ALL_EVENTS, MS2ST(10));

		if(events & EventFlags::EVENT_RADIO_RESET) {
			XPCC_LOG_DEBUG .printf("Si4432 RADIO Reset! Reinitializing\n");
			logRadioError(3);
			initRadioRegisters();
		}

		if(events & EventFlags::EVENT_RX_COMPLETE) {
			if(rxDataLen) {


				if(headerFlags() & PacketFlags::PACKET_RC) {
					RCPacket* pkt = (RCPacket*) rxBuf;
					printf("got rc packet %d\n", headerId());
				}


				rxDataLen = 0;
			}
		}

		if(checkTimer.isExpired()) {
			if(!checkRegistersValid()) {
				XPCC_LOG_DEBUG .printf("Radio register check failed, reinit!\n");
				logRadioError(4);
				initRadioRegisters();
			}
		}

//		if (!transmitting()) {
//			//make sure we are in RX mode always if not transmitting anything
//			setModeRx();
//
//			if (rxDataLen) {
//				noiseFloor = ((uint16_t) noiseFloor * 31 + rssiRead()) / 32;
//
//				uint8_t* buf = rxBuf;
//				uint8_t len = rxDataLen;
//
//				if (len >= sizeof(Packet)) {
//					Packet* inPkt = (Packet*) buf;
//
//					switch (inPkt->id) {
//					case PACKET_RC:
//						if(len >= sizeof(RCPacket)) {
//							rcData = *((RCPacket*) inPkt);
//							rcPacketTimestamp = Clock::now();
//
//							uint8_t payload_len = len - sizeof(RCPacket);
//							if(payload_len) {
//								if(inPkt->ackSeq == seq) {
//									rxbuf.write(buf+sizeof(RCPacket), payload_len);
//								} else {
//									XPCC_LOG_DEBUG
//									.printf("discard duplicate seq:%d ackSeq:%d\n", seq, inPkt->ackSeq);
//
//								}
//							}
//						}
//
//						break;
//					case PACKET_RF_PARAM_SET: {
//						RadioCfgPacket* cfg = (RadioCfgPacket*) inPkt;
//
//						printf("--Radio parameters received--\n");
//						printf("Freq %d\n", cfg->frequency/1000);
//						printf("AfcPullIn %d\n", cfg->afcPullIn/1000);
//						printf("Modem setting %d\n", cfg->modemCfg);
//						printf("FH Channels %d\n", cfg->fhChannels);
//						printf("TX Power %d\n", cfg->txPower);
//						setModeIdle();
//						if (radio_cfg.frequency != cfg->frequency/1000) {
//							setFrequency(cfg->frequency/1000, cfg->afcPullIn/1000);
//						}
//
//						if (cfg->modemCfg != radio_cfg.modemCfg.get()) {
//							setModemConfig((RH_RF22::ModemConfigChoice) cfg->modemCfg);
//						}
//
//						if (radio_cfg.txPower.get() != cfg->txPower) {
//							setTxPower(cfg->txPower);
//						}
//
//						radio_cfg.fhChannels.set_and_save_ifchanged(cfg->fhChannels);
//						setModeRx();
//
//					}
//						break;
//					}
//
//					//received packet, send data back
//					//if no data is available, send only ack
//					if (inPkt->id >= PACKET_RC) {
//						//lastAckSeq = inPkt->ackSeq; //set last acknowledged ours packet
//						if (radio_cfg.fhChannels)
//							setFHChannel((inPkt->seq ^ 0x55) % radio_cfg.fhChannels);
//
//						//dbgclr();
//						//printf("*\n");
//						sendAck(inPkt);
//
//					}
//				}
//
//				rxDataLen = 0;
//			}
//			//XPCC_LOG_DEBUG .dump_buffer(buf, len);
//		}
	}
}

bool Radio::init() {
	if(!hwInitialized) {
		initDMA();

		if(!RH_RF22::HWinit()) {
			AP_HAL::panic("Radio HW init failed");
			return false;
		}
	}
	hwInitialized = true;

	thread_irq = chThdCreateStatic(_irq_wa, sizeof(_irq_wa), NORMALPRIO+3, _irq_entry, this);
	thread_main = chThdCreateStatic(_main_wa, sizeof(_main_wa), NORMALPRIO, _main_entry, this);

	return true;
}

bool Radio::initRadioRegisters() {
	if(!RH_RF22::init()) {
		return false;
	}

	radio_cfg.frequency.load();
	radio_cfg.modemCfg.load();
	radio_cfg.txPower.load();
	radio_cfg.fhChannels.load();

	setFHStepSize(10);
	setFrequency(radio_cfg.frequency.get());
	setTxPower(radio_cfg.txPower.get());
	setModemConfig((RH_RF22::ModemConfigChoice)radio_cfg.modemCfg.get());

	updateRegisterSentinel();

	setModeRx();
	XPCC_LOG_DEBUG .printf("Radio initialized (f:%d)\n", radio_cfg.frequency.get());
}

bool Radio::sendAck(Packet* inPkt) {
//	Packet* out = (Packet*)txBuf;
//
//	if((uint16_t)radio_cfg.maxFragment > (RH_RF22_MAX_MESSAGE_LEN - sizeof(Packet))) {
//		radio_cfg.maxFragment = RH_RF22_MAX_MESSAGE_LEN - sizeof(Packet);
//	} else if((uint16_t)radio_cfg.maxFragment < 32) {
//		radio_cfg.maxFragment = 32;
//	}
//	uint16_t maxFrag = radio_cfg.maxFragment;
//
//	out->noise = getNoiseFloor();
//	out->rssi = getRssi();
//	//if last sent packet was a data packet, retry transmission
//	if(inPkt->ackSeq != seq && out->id == PACKET_DATA) {
//		//retry last data transmission
//		printf("Retry id:%d len:%d\n", out->id, dataLen);
//		out->seq = ++seq;
//		out->ackSeq = inPkt->seq;
//
//		RH_RF22::send(txBuf, dataLen);
//	} else {
//		uint16_t txavail = txbuf.bytes_used();
//		out->seq = ++seq;
//		out->ackSeq = inPkt->seq; //acknowledge received packet
//		out->id = PACKET_DATA;
//
//		if(txavail) {
//			uint8_t* buf = txBuf + sizeof(Packet);
//
//			for(int i = 0; i < std::min(maxFrag, txavail); i++) {
//				*buf++ = popTx();
//			}
//
//			dataLen = buf - txBuf;
//			//printf("Send data %d\n", dataLen);
//			RH_RF22::send(txBuf, dataLen);
//
//		} else {
//			out->id = PACKET_ACK; //no data to send, send ack
//			RH_RF22::send(txBuf, sizeof(Packet));
//		}
//	}

	return true;
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

		chEvtSignal(thread_main, (eventmask_t)EventFlags::EVENT_RX_COMPLETE);
	}
}

void Radio::handleInterrupt() {
	chSysLockFromISR();
	chEvtSignalI(thread_irq, EVENT_IRQ);
	chSysUnlockFromISR();
}

void Radio::handleTxComplete() {
	setModeRx();
	chEvtSignal(thread_main, (eventmask_t)EventFlags::EVENT_TX_COMPLETE);
}

void Radio::handleReset() {
	chEvtSignal(thread_main, (eventmask_t)EventFlags::EVENT_RADIO_RESET);
}
