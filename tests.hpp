/*
 * tests.hpp
 *
 *  Created on: Aug 24, 2016
 *      Author: walmis
 */

#ifndef TESTS_HPP_
#define TESTS_HPP_


#ifdef DEBUG
template <typename Task, size_t stack_size>
class ChTask : public Task, chibios_rt::BaseStaticThread<stack_size> {
public:
	template<typename ... Args>
	ChTask(Args ... args) :
			Task(args...) {
	}

protected:
	void handleInit() {
		chibios_rt::BaseStaticThread<stack_size>::start(NORMALPRIO);
		this->Task::handleInit();
	}

	void handleTick() override {
		chThdYield();
	}

	void _yield(uint16_t timeAvailable) override {
		chThdYield();
	}

	void main() override {
		while(1) {
			this->Task::handleTick();
			_yield(0);
		}
	}

};



class T2: TickerTask {

	void handleInit() {
		//hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&T2::test, void));
	}

	void test() {
		uint8_t data2[14];
		uint8_t addr = 0x3B;

		xpcc::I2cWriteReadTransaction t;
		t.initialize(0x68, &addr, 1, data2, 14);

		xpcc::stm32::I2cMaster1::start(&t);

		t.wait();

		if(t.getState() == I2cWriteReadTransaction::AdapterState::Idle) {
//					XPCC_LOG_DEBUG .dump_buffer(data2, 14);
//
//					XPCC_LOG_DEBUG .printf("A %d %d %d\n",
//							(int16_t)__builtin_bswap16(*(int16_t*)&data2[0]),
//							(int16_t)__builtin_bswap16(*(int16_t*)&data2[2]),
//							(int16_t)__builtin_bswap16(*(int16_t*)&data2[4]));

			//XPCC_LOG_DEBUG .printf("G %d %d %d\n");
		} else {
			XPCC_LOG_DEBUG .printf("error\n");
		}
	}

	void handleTick() {
		int c;

		static uint8_t b[512] __aligned(4);

		if ((c = uart1.read()) >= 0) {
			if (c == 'r') {
				NVIC_SystemReset();
			}

			if (c == 'y') {
				xpcc::I2cWriteTransaction adapter;

				uint8_t buf[3] = { 0x0F };

				XPCC_LOG_DEBUG << "Scanning i2c bus\n";
				for (int i = 0; i < 128; i++) {
					adapter.initialize(i, buf, 1);
					if (!I2cMaster1::start(&adapter)) {
						XPCC_LOG_DEBUG.printf("start failed\n");
					}
					while (adapter.isBusy())
						;

					if (I2cMaster1::getErrorState()
							!= xpcc::I2cMaster::Error::AddressNack) {
						XPCC_LOG_DEBUG.printf("Found device @ 0x%x %d %d\n", i,
								I2cMaster1::getErrorState(),
								adapter.getState());
					}
				}
			}

			if(c == '4') {
				radio.reset();
			}

			if (c == 't') {
				I2cWriteReadTransaction adapter;
				uint8_t data1 = 0;
				uint8_t c = 0x75;

				uint8_t data2 = 0;
				if (hal.i2c->readRegister(0x68, 0x0, &data1) == 1) {
					XPCC_LOG_DEBUG.printf("failed\n");
				}

				XPCC_LOG_DEBUG.printf("%x\n", data1);

			}

			if (c == 'x') {
				uint8_t data2[12];
				//while(1) {
				if (hal.i2c->readRegisters(0x68, 0x0C, 2, data2) == 1) {
					XPCC_LOG_DEBUG.printf("failed\n");
				}
				//}
				XPCC_LOG_DEBUG.printf("%x\n", data2[0]);
			}

			if (c == 'z') {
				uint8_t data2[12];
				//while(1) {
				if (hal.i2c->readRegisters(0x68, 0x0C, 4, data2) == 1) {
					XPCC_LOG_DEBUG.printf("failed\n");
				}
				//}
				XPCC_LOG_DEBUG.printf("%x\n", data2[0]);
			}

			if (c == 's') {
				memset(b, 0, 512);

//			uint32_t c = stm32::GPTimer5::getValue();
//			for (int i = 0; i < 10; i++) {
//				sd.readSingleBlock(b, i);
//			}
//			c = stm32::GPTimer5::getValue() - c;
//			XPCC_LOG_DEBUG.printf("time %d\n", c);

				//uint32_t c = stm32::GPTimer5::getValue();
				//for(int i = 0; i < (1024*1024)/512; i++) {
				//	sd.readSingleBlock(b, i);
				//}
				//c = stm32::GPTimer5::getValue() - c;
				//XPCC_LOG_DEBUG .printf("time %d\n", c );

				//sdCard.doRead(b, 1, 2);
				sdCard.doRead(b, 0, 1);
				XPCC_LOG_DEBUG.dump_buffer(b, 512);

			}

			if (c == 'b') {
				__disable_irq();
				 *((unsigned long *)0x2000FFF0) = 0xDEADBEEF;
				NVIC_SystemReset();

			}

			if (c == '2') {

				sdCard.writeStart(1, 5);


				memset((uint8_t*) b, 0x11, 512);
				sdCard.writeData(b);

				memset((uint8_t*) b, 0x22, 512);
				sdCard.writeData(b);

				memset((uint8_t*) b, 0x33, 512);
				sdCard.writeData(b);

				memset((uint8_t*) b, 0x44, 512);
				sdCard.writeData(b);

				memset((uint8_t*) b, 0x55, 512);
				sdCard.writeData(b);

				sdCard.writeStop();

				sdCard.writeStart(1, 1);
				memset((uint8_t*) b, 0x11, 512);
				sdCard.writeData(b);
				sdCard.writeStop();

			}

			if (c == '3') {

				memset((uint8_t*) b, 0x11, 512);

				for(int i = 5; i < 15; i++) {
					sdCard.writeBlock(i, b);
				}

			}

			if (c == 'm') {
				sdCard.readStart(1);

				uint32_t c = stm32::GPTimer5::getValue();
				for (int i = 0; i < 100; i++) {
					sdCard.readData(b, i);
				}
				c = stm32::GPTimer5::getValue() - c;
				XPCC_LOG_DEBUG.printf("time %d\n", c);

				sdCard.readData(b, 512);

				XPCC_LOG_DEBUG.dump_buffer(b, 512);

				sdCard.readData(b, 512);
				XPCC_LOG_DEBUG.dump_buffer(b, 512);

				sdCard.readData(b, 512);
				XPCC_LOG_DEBUG.dump_buffer(b, 512);

				sdCard.readStop();

			}

			if (c == 'i') {
				EXTI->SWIER |= (1 << 0);
			}

			if(c == '[') {
				uint8_t buf[255];
				memset(buf, 0, 255);

				hal.storage->read_block(&buf, 2000, 16);
				XPCC_LOG_DEBUG .dump_buffer(buf, 16);
			}

			if(c == 'k') {
				while(1) {
					XPCC_LOG_DEBUG << 'a';
				}

			}

			if(c == '>') {
				((XpccHAL::Storage*)hal.storage)->block_cache.dump();
			}
			if(c == '*') {
				volatile float testas = 0.45f;
				testas /= 0.0f;
			}

			if(c == ']') {
				static uint8_t g = 0;
				g++;
				uint8_t buf[255];
				for(int i =0; i < 255; i++) {
					buf[i] = g;
				}
				XPCC_LOG_DEBUG .dump_buffer(buf, 16);
				hal.storage->write_block(2000, buf, 16);
			}

			if(c == '0') {
				uint8_t data2[14];
				uint8_t addr = 0x3B;

				xpcc::I2cWriteReadTransaction t;
				t.initialize(0x68, &addr, 1, data2, 14);

				xpcc::stm32::I2cMaster1::start(&t);

				t.wait();

				if(t.getState() == I2cWriteReadTransaction::AdapterState::Idle) {
					XPCC_LOG_DEBUG .dump_buffer(data2, 14);

					XPCC_LOG_DEBUG .printf("A %d %d %d\n",
							(int16_t)__builtin_bswap16(*(int16_t*)&data2[0]),
							(int16_t)__builtin_bswap16(*(int16_t*)&data2[2]),
							(int16_t)__builtin_bswap16(*(int16_t*)&data2[4]));

					XPCC_LOG_DEBUG .printf("G %d %d %d\n",
							(int16_t)__builtin_bswap16(*(int16_t*)&data2[8]),
							(int16_t)__builtin_bswap16(*(int16_t*)&data2[10]),
							(int16_t)__builtin_bswap16(*(int16_t*)&data2[12]));

					//XPCC_LOG_DEBUG .printf("G %d %d %d\n");
				} else {
					XPCC_LOG_DEBUG .printf("error\n");
				}

			}

		}
		chThdYield();
	}


};

T2 test;
#endif


#endif /* TESTS_HPP_ */
