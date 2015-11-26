/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* 
   DataFlash logging - file oriented variant

   This uses posix file IO to create log files called logs/NN.bin in the
   given directory
 */

#include <AP_HAL.h>

#include "DataFlash_Xpcc.h"
#include <xpcc/architecture.hpp>
#include <xpcc/processing.hpp>
#include <fatfs/diskio.h>
#include <fatfs/ff.h>
#include "../pindefs.hpp"

using namespace xpcc;

extern const AP_HAL::HAL& hal;
extern xpcc::fat::FileSystem fs;

#define MAX_LOG_FILES 500U
#define DATAFLASH_PAGE_SIZE 1024UL

void DataWriterThread::main() {
	if(!buffer.allocate(16*1024)) {
		AP_HAL::panic("Failed to allocate logger buffer");
	}

	while(1) {
		static PeriodicTimer<> fsync_timeout(500);
		//wait for data
		dataAvail.wait(500);
		if(stop) {
			file = 0;
			stop = false;
		}

		if(file) {
			static uint16_t written;

			while((buffer.bytes_used() >= sizeof(tmpBuffer))) {
				LedRed::set();
				int16_t n_read = buffer.read(tmpBuffer, sizeof(tmpBuffer));

				if(n_read > sizeof(tmpBuffer)) {
					XPCC_LOG_DEBUG .printf("n_read > sizeof(tmpBuffer)\n");
					while(1);
				}

				if(guard != 0xDEADBEEF) {
					XPCC_LOG_DEBUG .printf("buffer overflow\n");
#ifdef DEBUG
					while(1);
#endif
					return;
				}

				if(n_read > 0) {
					if(file->write(tmpBuffer, n_read) == (size_t)-1) {
						//error
						hal.console->print("Dataflash write error\n");
						file = 0;
						stopWrite();
						LedRed::reset();
						break;
					}
					written += n_read;
				}
				if(fsync_timeout.isExpired()) {
					file->flush();

					//size_t cur = file->ftell();
					//file->lseek(cur+written*4);
					//file->lseek(cur);
					written = 0;
				}
				LedRed::reset();
			}
//			if(t.isExpired()) {
//				file->flush();
//			}
		}
	}
}

bool DataWriterThread::isActive() {
	return file != 0;
}

void DataWriterThread::startWrite(xpcc::fat::File *new_file) {
	stopWrite();

	file = new_file;
}


void DataWriterThread::stopWrite() {
	stop = true;

	dataAvail.signal(); //wakeup thread

	while(stop) { //wait until stopped
		yield();
	}
}


bool DataWriterThread::write(uint8_t* data, size_t size) {
	if(!file)
		return false;

	if(buffer.bytes_free() < size) {
		//lets try a yield, maybe the writer will free up some space
		yield();
		if(buffer.bytes_free() < size) {
			//nope, okay then fail
			return false;
		}
	}

	buffer.write(data, size);
	dataAvail.signal();
	return true;
}

/*
  constructor
 */
void DataFlash_Xpcc::Init(const struct LogStructure *structure, uint8_t num_types)
{
	DataFlash_Backend::Init(structure, num_types);
	_writes_enabled = true;
	log_write_started = false;

    XPCC_LOG_INFO << "LOG: Init\n";

    storage_lock = 1;

    uint32_t sectors;
    if(fs.volume()->doIoctl(GET_SECTOR_COUNT, &sectors) == RES_OK && sectors == 0) {
    	hal.console->println("LOG: No card inserted");
    	return;
    }

    fat::Directory dir;
    FRESULT res;
    fat::FileInfo fil;

    num_logs = 0;

    res = dir.open("/apm");
    if(res == FR_OK) {
    	hal.console->println("LOG: Log directory /apm\n");

    	fat::File f;
    	if(f.open("/apm/lastlog.txt", "r") == FR_OK) {
    		char data[16];
    		uint32_t n = f.read((uint8_t*)data, 16);
    		if(n > 0) {
    			data[n] = 0;
    			last_log = atol(data);
    			hal.console->printf("LOG: Last log #%d\n", last_log);
    		}

    	} else {
    		f.open("/apm/lastlog.txt", "w");
    		f.write('0');
    		f.close();
    	}

    	while(dir.readDir(fil) == FR_OK && !fil.eod()) {
    		if(strstr(fil.getName(), ".bin")) {
    			//hal.console->println( fil.getName() );
    			num_logs++;
    		}
    	}

    } else {
    	res = fat::FileSystem::mkdir("/apm");
    	if(res != FR_OK) {
    		hal.console->println("LOG: Unable to create /apm log directory");
    	}
    	last_log = 0;
    }

    dir.close();

    storage_lock = 0;

    writer.start(NORMALPRIO);
}

fat::File* DataFlash_Xpcc::openLog(uint16_t log_num, char* mode) {
	fat::Directory dir;

	dir.open("/apm");
	int count = 0;

	fat::File *file = 0;

	dir.readDir([&count,log_num,&file, mode](fat::FileInfo* info) {
		if(count == log_num) {
			StringStream<32> name;
			name << "/apm/" << info->getName();

			file = new fat::File;
			if(file->open(name.buffer, mode) != FR_OK) {
				delete file;
				file = 0;
			}

			return;
		}
		count++;
	});

	dir.close();

	return file;
}

bool DataFlash_Xpcc::NeedErase(void){
	return false;
}

bool DataFlash_Xpcc::NeedPrep() {
	return false;
}

void DataFlash_Xpcc::Prep() {
    if (hal.util->get_soft_armed()) {
        // do not want to do any filesystem operations while we are e.g. flying
        return;
    }
}

uint16_t DataFlash_Xpcc::bufferspace_available() {
	return writer.bytesAvailable();
}

bool DataFlash_Xpcc::WritePrioritisedBlock(const void* pBuffer, uint16_t size,
		bool is_critical) {

	if(!file || !_writes_enabled || !storage_lock)
		return false;

    if (! WriteBlockCheckStartupMessages()) {
        return false;
    }

    if(is_critical) {
    	while(writer.bytesAvailable() < size && writer.isActive()) {
    		yield();
    	}
    }

    WriteBlock(pBuffer, size);

}


void DataFlash_Xpcc::WriteBlock(const void* pBuffer, uint16_t size) {
//	if(!storage_lock && !hal.gpio->usb_connected()) {
//		storage_lock = true;
//	}
	if(!file || !_writes_enabled || !storage_lock)
		return;

	static uint32_t count;
	static uint32_t dropped = 0;
	count += size;
#ifdef DEBUG
	static xpcc::PeriodicTimer<> t(1000);
	if(t.isExpired()) {
		XPCC_LOG_DEBUG .printf("log wr %d b/s (buf avail %d, dropped %d)\n", count,
				writer.bytesAvailable(), dropped);
		count = 0;

	}
#endif


	//if usb is connected, stop writing logs and mount msd storage
	if(file && file->isOpened() && hal.gpio->usb_connected()) {
		XPCC_LOG_DEBUG .printf("USB Connected, stop log write\n");
		writeLock.lock();

		writer.stopWrite();
		file->close();

		storage_lock = 0;
		writeLock.unlock();
		return;
	}

	writeLock.lock();
	//XPCC_LOG_DEBUG .printf("Dataflash: data overrun\n");

	bool res = writer.write((uint8_t*)pBuffer, size);

	writeLock.unlock();

	if(!res) {
		dropped += size;
	}

}

uint16_t DataFlash_Xpcc::find_last_log(void) {
	XPCC_LOG_INFO << __PRETTY_FUNCTION__ << xpcc::endl;
	return last_log;
}

void DataFlash_Xpcc::get_log_boundaries(uint16_t log_num, uint16_t& start_page,
		uint16_t& end_page) {
	start_page = 0;

	uint32_t timestamp, size;

	get_log_info(log_num, size, timestamp);

	end_page = size / DATAFLASH_PAGE_SIZE;

	if(read_file) {
		read_file->close();
		delete read_file;
	}
}

void DataFlash_Xpcc::get_log_info(uint16_t log_num, uint32_t& size,
		uint32_t& time_utc) {

	fat::Directory dir;

	dir.open("/apm");
	int count = 0;

	dir.readDir([&count,log_num,&size,&time_utc](fat::FileInfo* info) {
		if(count == log_num) {
			size = info->getSize();
			time_utc = info->getUnixTimestamp();
		}
		count++;
	});

	dir.close();
}

int16_t DataFlash_Xpcc::get_log_data(uint16_t log_num, uint16_t page,
		uint32_t offset, uint16_t len, uint8_t* data) {
	//XPCC_LOG_INFO << __PRETTY_FUNCTION__ << xpcc::endl;

	if(!read_file) {
		read_file = openLog(log_num, "r");
		if(!read_file) {
			XPCC_LOG_DEBUG .printf("Failed to open log for reading\n");
			return -1;
		}
	}
	//TODO: implement offsets

	len = read_file->read(data, len);

	return len;
}

uint16_t DataFlash_Xpcc::get_num_logs(void) {
	return num_logs;
}

void DataFlash_Xpcc::LogReadProcess(uint16_t log_num,
        uint16_t start_page, uint16_t end_page,
        print_mode_fn printMode,
        AP_HAL::BetterStream *port) {
	XPCC_LOG_INFO << __PRETTY_FUNCTION__ << xpcc::endl;
}

void DataFlash_Xpcc::DumpPageInfo(AP_HAL::BetterStream* port) {
}

void DataFlash_Xpcc::ShowDeviceInfo(AP_HAL::BetterStream* port) {
}

void DataFlash_Xpcc::ListAvailableLogs(AP_HAL::BetterStream* port) {
}

uint16_t DataFlash_Xpcc::start_new_log(void) {

	if(hal.gpio->usb_connected()) {
		XPCC_LOG_DEBUG << "LOG: USB connected, not starting log\n";
		storage_lock = 0;
		return 0xFFFF;
	}

	storage_lock = 1;

	if(!file) {
		file = new xpcc::fat::File;
	} else {
		if(file->isOpened()) {
			file->close();
		}
	}

	last_log++;
	XPCC_LOG_DEBUG .printf("LOG: Starting Logging to %d.bin\n", last_log);

	StringStream<32> s;
	s << "/apm/" << last_log << ".bin";

	if(!file->open(s.buffer, "w") == FR_OK) {
		hal.console->printf("LOG: Failed to open %s for writing\n", s.buffer);
		writer.startWrite(0);
		storage_lock = 0;
		return 0xFFFF;
	} else {
		file->flush();

		//preallocate 256kb
		file->lseek(256*1024);
		file->lseek(0);

		file->flush();

		writer.startWrite(file);

		fat::File l;
		l.open("/apm/lastlog.txt", "w");
		l << last_log;
		l.close();
	}
	log_write_started = true;

	return last_log;
}

void DataFlash_Xpcc::ReadManufacturerID() {
}

bool DataFlash_Xpcc::CardInserted() {
	if(fs.volume()->doGetStatus() & STA_NODISK) {
		return false;
	}
	return true;
}

DataFlash_Xpcc* dataflash = 0;

//exported function
DataFlash_Backend* SKYFalcon_getDataflash(DataFlash_Class &front) {
	if(!dataflash) {
		dataflash = new DataFlash_Xpcc(front);
	}
	return dataflash;
}


