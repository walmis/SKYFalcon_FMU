/*
 * radio.cpp
 *
 *  Created on: Sep 24, 2014
 *      Author: walmis
 */

#include "radio.hpp"
#include "pindefs.hpp"

using namespace xpcc;

extern const AP_HAL::HAL& hal;

void Radio::main() {
	chibios_rt::BaseThread::setName("Radio");

	while(1) {
		if(int_event.wait(1)) {
			this->RH_RF22::handleInterrupt();
		}

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
		if(!RH_RF22::HWinit()) {
			hal.scheduler->panic("Radio init failed");
			return false;
		}
	}
	hwInitialized = true;

	RH_RF22::init();

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

	this->Thread::start(NORMALPRIO+1);

	return true;
}


bool Radio::sendAck(Packet* inPkt) {

	uint16_t txavail = txbuf.bytes_used();
	Packet* out = (Packet*)txBuf;

	if((uint16_t)radio_cfg.maxFragment > (RH_RF22_MAX_MESSAGE_LEN - sizeof(Packet))) {
		radio_cfg.maxFragment = RH_RF22_MAX_MESSAGE_LEN - sizeof(Packet);
	} else if((uint16_t)radio_cfg.maxFragment < 32) {
		radio_cfg.maxFragment = 32;
	}
	uint16_t maxFrag = radio_cfg.maxFragment;

	out->noise = getNoiseFloor();
	out->rssi = getRssi();

	if(inPkt->ackSeq != seq && out->id == PACKET_DATA) {
		//retry last data transmission
		printf("Retry\n");
		out->seq = ++seq;
		out->ackSeq = inPkt->seq;

		RH_RF22::send(txBuf, dataLen);
	} else {

		out->seq = ++seq;
		out->ackSeq = inPkt->seq; //acknowledge received packet
		out->id = PACKET_DATA;

		if((txavail >= maxFrag) || (latencyTimer.isExpired() && txavail)) {

			uint8_t* buf = txBuf + sizeof(Packet);
			for(int i = 0; i < std::min(maxFrag, txavail); i++) {
				*buf++ = txbuf.read();
			}

			dataLen = buf - txBuf;
			//printf("Send data %d\n", dataLen);
			RH_RF22::send(txBuf, dataLen);

			latencyTimer.restart(latency);
		} else {
			out->id = PACKET_ACK; //no data to send, send ack
			RH_RF22::send(txBuf, sizeof(Packet));
		}
	}

	return true;
}

uint8_t Radio::spiBurstWrite0(uint8_t reg, const uint8_t* src, uint8_t len) {

}

uint8_t Radio::spiBurstRead0(uint8_t reg, uint8_t* dest, uint8_t len) {

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
	}


}

void Radio::handleInterrupt() {
	int_event.signal();
}

void Radio::handleTxComplete() {
	setModeRx();
}

void Radio::handleReset() {
	XPCC_LOG_DEBUG .printf("RF22 Reset!\n");
}
