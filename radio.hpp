/*
 * radio.hpp
 *
 *  Created on: Sep 18, 2014
 *      Author: walmis
 */

#ifndef RADIO_HPP_
#define RADIO_HPP_

#include <xpcc/architecture.hpp>
#include "pindefs.hpp"
#include <RH_RF22.h>
#include "AP_Radio.hpp"

enum PacketType {
	PACKET_RC = 100,
	PACKET_RF_PARAM_SET,
	PACKET_DATA_FIRST, //first data fragment
	PACKET_DATA, //data fragment
	PACKET_DATA_LAST, //last data fragment
	PACKET_ACK
};

struct Packet {
	uint8_t id = PACKET_RC;
	uint8_t seq; //sequence number
	uint8_t ackSeq; //rx acknowledged seq number
	int8_t rssi; // local rssi dBm
	int8_t noise; //local noise dBm
} __attribute__((packed));

struct RadioCfgPacket : Packet {
	RadioCfgPacket() {
		id = PACKET_RF_PARAM_SET;
	}
	uint32_t frequency;
	uint32_t afcPullIn;
	uint8_t modemCfg;
	uint8_t fhChannels;
	uint8_t txPower;
} __attribute__((packed));

struct RCPacket : Packet {
	uint16_t channels[16];
} __attribute__((packed));

class Radio final : chibios_rt::BaseStaticThread<512>, public RH_RF22, public BufferedIODevice {
public:
	typedef chibios_rt::BaseStaticThread<512> Thread;

	Radio() : RH_RF22(nRadioSel::Id, RadioIrq::Id),
			BufferedIODevice(256, 256) {}

	bool init();

	bool isSendingPacket() {
		return dataLen != 0;
	}

    inline uint16_t getRxBad() {
    	return _rxBad;
    }

    inline uint16_t getRxGood() {
    	return _rxGood;
    }

    inline uint16_t getTxGood() {
    	return _txGood;
    }

    int8_t getNoiseFloor() {
    	return ((noiseFloor * 100) / 190) - 127;
    }

    int8_t getRssi() {
    	return ((rssi * 100) / 190) - 127;
    }

    void setFrequency(uint32_t f_khz, uint32_t afc_khz = 50) {
    	radio_cfg.frequency.set_and_save_ifchanged(f_khz);

    	RH_RF22::setFrequency(f_khz * 1000, afc_khz*1000);
    }

    void setTxPower(uint8_t pow) {
    	radio_cfg.txPower.set_and_save_ifchanged(pow);

    	RH_RF22::setTxPower(pow);
    }

    void setModemConfig(RH_RF22::ModemConfigChoice cfg) {
    	radio_cfg.modemCfg.set_and_save(cfg);

    	RH_RF22::setModemConfig(cfg);
    }

    const uint8_t latency = 1;
    xpcc::Timeout<> latencyTimer;

    RCPacket rcData;
    xpcc::Timestamp rcPacketTimestamp;

    friend class CmdTerminal;

    static const struct AP_Param::GroupInfo var_info[];
protected:

    void main() override;

    void handleInterrupt() override;

	void handleTxComplete() override;
	void handleRxComplete() override;
	void handleReset() override;
	bool sendAck(Packet* inPkt);

	xpcc::Event int_event;

	uint8_t txBuf[255];
	uint8_t dataLen		= 0; //packet size
	uint8_t dataPos		= 0;

	uint8_t rxBuf[255];
	volatile uint8_t rxDataLen;

	//uint8_t lastAckSeq; //last acknowledged sequence number
	//uint8_t lastSeq;

	uint8_t noiseFloor	= 0;
	uint8_t rssi		= 0;

	uint32_t numRetries	= 0;
	uint8_t seq			= 0;

	bool hwInitialized	= false;

    bool transmitting() {
    	return mode() == RHModeTx;
    }

    bool idle() {
    	return mode() == RHModeIdle;
    }

private:

    uint8_t spiBurstWrite0(uint8_t reg, const uint8_t* src, uint8_t len);
    uint8_t spiBurstRead0(uint8_t reg, uint8_t* dest, uint8_t len);

};

extern Radio radio;

#endif /* RADIO_HPP_ */
