/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* 
   DataFlash logging - file oriented variant

   This uses posix file IO to create log files called logNN.dat in the
   given directory
 */

#pragma once

#include <DataFlash.h>
#include <xpcc/processing.hpp>
#include <xpcc/driver/storage/fat.hpp>

class DataWriterThread : public chibios_rt::BaseStaticThread<512> {
public:

	void main();

	void startWrite(xpcc::fat::File *file);

	bool write(uint8_t* data, size_t size);

	void stopWrite();

	uint16_t bytesAvailable() {
		return buffer.bytes_free();
	}

protected:

	uint8_t tmpBuffer[1024];
	IOBuffer buffer;
	xpcc::fat::File* file;
	xpcc::Event dataAvail;
	volatile bool stop;
};

class DataFlash_Xpcc : public DataFlash_Class
{
public:

    void WriteBlock(const void *pBuffer, uint16_t size);
    uint16_t find_last_log(void);
    void get_log_boundaries(uint16_t log_num, uint16_t & start_page, uint16_t & end_page);
    void get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc);
    int16_t get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data);
    uint16_t get_num_logs(void);

    void LogReadProcess(uint16_t log_num,
				uint16_t start_page, uint16_t end_page,
				print_mode_fn printMode,
				AP_HAL::BetterStream *port);
    void DumpPageInfo(AP_HAL::BetterStream *port);
    void ShowDeviceInfo(AP_HAL::BetterStream *port);
    void ListAvailableLogs(AP_HAL::BetterStream *port);

    uint16_t start_new_log(void);

    xpcc::fat::File* openLog(uint16_t log_num, char* mode);

    void        Init(const struct LogStructure *structure, uint8_t num_types);
    void        ReadManufacturerID();
    bool        CardInserted();

    bool NeedErase(void);
    void EraseAll() {}

    bool ReadBlock(void *pkt, uint16_t size) {
    	return false;
    };

    void setBlockingWrites(bool s) {
    	blockingWrites = s;
    }


protected:
    xpcc::fat::File *file;
    xpcc::fat::File *read_file;
    DataWriterThread writer;
    chibios_rt::Mutex writeLock;

    uint16_t last_log;
    uint16_t num_logs;

    bool blockingWrites = true;
};

extern DataFlash_Xpcc dataflash;


