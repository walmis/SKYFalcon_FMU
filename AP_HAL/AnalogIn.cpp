#include <xpcc/architecture.hpp>
#include "AnalogIn.h"

using namespace XpccHAL;
using namespace xpcc::stm32;

extern const AP_HAL::HAL& hal;

uint8_t AnalogSource::_ch_cnt = 0;

AnalogSource::AnalogSource(int16_t chan, AnalogIn* parent) : _chan(-1), _chan_idx(-1)
{
	count = 0;
	sum = 0;
	_voltage_avg = 0;
	_voltage_latest = 0;

	if(chan != -1) {
		set_pin(chan);
	}
}


void AnalogSource::set_pin(uint8_t p)
{
	if(p == 255) return;
	XPCC_LOG_DEBUG .printf("adding adc channel %d\n", p);
	//register channel with ADC
	if(_chan_idx == -1) {
		if(_ch_cnt == 0) {
			Adc1::setChannel((Adc1::Channel)p, Adc1::SampleTime::Cycles112);
		} else {
			Adc1::addChannel((Adc1::Channel)p, Adc1::SampleTime::Cycles112);
		}

		_chan = p;
		_chan_idx = _ch_cnt;
		_ch_cnt++;

	} else {
		//update existing channel
		Adc1::setChannel(_chan_idx, (Adc1::Channel)p, Adc1::SampleTime::Cycles112);
		_chan = p;
	}

}


float AnalogSource::read_average() {
    return _voltage_avg;
}

float AnalogSource::voltage_average() {
	//hal.console->printf("%.3f\n", _voltage_avg);

    return _voltage_avg;
}

float AnalogSource::voltage_latest() {
    return read_latest();
}

float AnalogSource::read_latest() {
	if(_chan == -1)
		return -1.0;

    return _voltage_latest;
}


void AnalogSource::_update() {
	if(_chan_idx == -1) return;

	uint16_t rawValue = static_cast<AnalogIn*>(hal.analogin)->samples[_chan_idx];

	_voltage_latest = (rawValue) * (3.3f / 4096.0f);

	_voltage_avg = (_voltage_avg*99 + _voltage_latest) / 100.0;
}



void AnalogSource::set_stop_pin(uint8_t p)
{}

void AnalogSource::set_settle_time(uint16_t settle_time_ms)
{}

AnalogIn::AnalogIn() :
		dmastream(dma::Stream::DMA2_0),
		num_channels(0)
{
	samples = new uint16_t[max_channels];
	for(int i = 0; i < max_channels; i++) {
		channels[i] = 0;
		samples[i] = 0;
	}
}

void AnalogIn::init(void* machtnichts)
{
	Adc1::initialize();

	Adc1::enableTemperatureRefVMeasurement();

    xpcc::stm32::dma::Config dmacfg;
	//initialize dma config
	dmacfg.channel(dma::Channel::Channel_0)
			->bufferSize(num_channels)
			->memoryDataSize(dma::MemoryDataSize::HalfWord)
			->peripheralDataSize(dma::PeripheralDataSize::HalfWord)
			->memoryInc(dma::MemoryInc::Enable)
			->periphBaseAddress((uint32_t)&ADC1->DR)
			->xferDirection(dma::XferDir::PeripheralToMemory)
			->memory0BaseAddress((uint32_t)&samples[0])
			->fifoMode(dma::FIFOMode::Disable)
			->priority(dma::Prioriy::Low);

	vref = channel((uint8_t)Adc1::Channel::InternalReference); //samples[0]
	vdd = channel((uint8_t)Adc1::Channel::BatDiv2); //samples[1]

	dmastream.init(dmacfg);

	Adc1::setScanMode(true);

	hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AnalogIn::_tick, void));
}

void AnalogIn::_tick()
{
	for(int i = 0; i < num_channels; i++) {
		channels[i]->_update();
	}

	dmastream.clearInterruptFlags(dma::IntFlag::All);
	dmastream.setCurrDataCounter(num_channels);

	Adc1::acknowledgeInterruptFlag(Adc1::InterruptFlag::All);

	Adc1::setDMAMode(true);
	dmastream.enable(true);

	Adc1::startConversion();
}


AP_HAL::AnalogSource* AnalogIn::channel(int16_t n) {

	for(int i = 0; i < max_channels; i++) {
		if(channels[i] == 0) {
			channels[i] = new AnalogSource(n, this);

			num_channels++;
			return channels[i];
		}
	}
	hal.console->println("Out of analog channels");
	return 0;
}

float AnalogIn::board_voltage(void) {
	return 4096 * 1.21f / samples[0];
}



