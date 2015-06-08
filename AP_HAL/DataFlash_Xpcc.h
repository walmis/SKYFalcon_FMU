/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* 
   DataFlash logging - file oriented variant

   This uses posix file IO to create log files called logNN.dat in the
   given directory
 */

#ifndef DataFlash_File_h
#define DataFlash_File_h

#include <DataFlash.h>
#include <xpcc/driver/storage/fat.hpp>

class DataWriter : xpcc::TickerTask {
public:
	void startWrite(xpcc::fat::File *file);

	bool write(uint8_t* data, size_t size);

	void stopWrite();

	uint16_t bytesAvailable() {
		return buffer.bytes_free();
	}

protected:
	void handleTick();
	void handleInit();

	uint8_t tmpBuffer[512];
	IOBuffer buffer;
	xpcc::fat::File* file;
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
                                void (*printMode)(AP_HAL::BetterStream *port, uint8_t mode),
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

protected:
    xpcc::fat::File *file;
    xpcc::fat::File *read_file;
    DataWriter writer;

    uint16_t last_log;
    uint16_t num_logs;
};

#endif // DataFlash_File_h

