/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* 
   DataFlash logging - file oriented variant

   This uses posix file IO to create log files called logs/NN.bin in the
   given directory
 */

#include <AP_HAL.h>

#include "DataFlash_Xpcc.h"
#include <xpcc/architecture.hpp>
#include <fatfs/diskio.h>
#include <fatfs/ff.h>

using namespace xpcc;

extern const AP_HAL::HAL& hal;
extern xpcc::fat::FileSystem fs;
extern volatile bool storage_lock;

#define MAX_LOG_FILES 500U
#define DATAFLASH_PAGE_SIZE 1024UL

void DataWriter::startWrite(xpcc::fat::File *file) {
	this->file = file;
	if(!file) {
		stopTask(); //stop task
	} else {
		startTask(); //resume task
	}
}

void DataWriter::handleInit() {
	if(!buffer.allocate(1024*8)) {
		hal.scheduler->panic("Failed to allocate logger buffer");
	}
}

void DataWriter::stopWrite() {
	if(file) {
		//write all remaining data
		while(buffer.bytes_used()) {
			handleTick();
		}
	}
	stopTask(); //stop task
}

void DataWriter::handleTick() {
	static PeriodicTimer<> t(500);
	if(file && storage_lock) {
		while(buffer.bytes_used()) {
			int16_t n_read = buffer.read(tmpBuffer, sizeof(tmpBuffer));
			if(n_read > 0) {
				if(file->write(tmpBuffer, n_read) == -1) {
					//error
					hal.console->print("Dataflash write error\n");
					file = 0;
					stopWrite();
					return;
				}
			}
		}
		if(t.isExpired()) {
			file->flush();
		}
	}
}

bool DataWriter::write(uint8_t* data, size_t size) {
	if(!file)
		return false;

	if(buffer.bytes_free() < size) {
		hal.console->printf("Dataflash blocking write!\n");
	}
	while(buffer.bytes_free() < size) {
		xpcc::yield();
	}

	buffer.write(data, size);
	return true;
}

/*
  constructor
 */
void DataFlash_Xpcc::Init(const struct LogStructure *structure, uint8_t num_types)
{
    DataFlash_Class::Init(structure, num_types);
    XPCC_LOG_INFO << "dataflash init\n";

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
    			hal.console->println( fil.getName() );
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

    //if usb is connected release lock
    if(hal.gpio->usb_connected())
    	storage_lock = 0;
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

void DataFlash_Xpcc::WriteBlock(const void* pBuffer, uint16_t size) {

	static xpcc::PeriodicTimer<> t(1000);
	static uint32_t count;
	count += size;
	if(t.isExpired()) {
		XPCC_LOG_DEBUG .printf("log wr %d b/s\n", count);
		count = 0;
	}

	//if usb is connected, stop writing logs and mount msd storage
	if(file && file->isOpened() && hal.gpio->usb_connected()) {
		XPCC_LOG_DEBUG .printf("usb detected, stop log write\n");
		writer.stopWrite();
		file->close();

		storage_lock = 0;
		return;
	}

	if(storage_lock) {


		//XPCC_LOG_DEBUG .printf("Dataflash: data overrun\n");

		writer.write((uint8_t*)pBuffer, size);
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

void DataFlash_Xpcc::LogReadProcess(uint16_t log_num, uint16_t start_page,
		uint16_t end_page,
		void (*printMode)(AP_HAL::BetterStream* port, uint8_t mode),
		AP_HAL::BetterStream* port) {
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
		XPCC_LOG_DEBUG << "USB connected, dont start log\n";
		storage_lock = 0;
		return 0xFFFF;
	}

	storage_lock = 1;

	XPCC_LOG_DEBUG .printf("start new log\n");

	if(!file) {
		file = new xpcc::fat::File;
	} else {
		if(file->isOpened()) {
			file->close();
		}
	}

	last_log++;

	StringStream<32> s;
	s << "/apm/" << last_log << ".bin";

	if(!file->open(s.buffer, "w") == FR_OK) {
		hal.console->printf("LOG: Failed to open %s for writing\n", s.buffer);
		writer.startWrite(0);
		storage_lock = 0;
		return 0xFFFF;
	} else {
		file->flush();
		writer.startWrite(file);

		fat::File l;
		l.open("/apm/lastlog.txt", "w");
		l << last_log;
		l.close();
	}

	return 0;
}

void DataFlash_Xpcc::ReadManufacturerID() {
}

bool DataFlash_Xpcc::CardInserted() {
	if(fs.volume()->doGetStatus() & STA_NODISK) {
		return false;
	}
	return true;
}
