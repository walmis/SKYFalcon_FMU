
#ifndef __AP_HAL_EMPTY_UARTDRIVER_H__
#define __AP_HAL_EMPTY_UARTDRIVER_H__

#include <xpcc/architecture.hpp>
#include "AP_HAL_XPCC.h"

class XpccHAL::UARTDriver final: public AP_HAL::UARTDriver {
public:
    UARTDriver(xpcc::IODevice* device);
    /* Empty implementations of UARTDriver virtual methods */
    void begin(uint32_t b);
    void begin(uint32_t b, uint16_t rxS, uint16_t txS);
    void end();
    void flush();
    bool is_initialized();
    void set_blocking_writes(bool blocking);
    bool tx_pending();

    /* Empty implementations of Stream virtual methods */
    int16_t available();
    int16_t txspace();
    int16_t read();

    /* Empty implementations of Print virtual methods */
    size_t write(uint8_t c);
    size_t write(const uint8_t *buffer, size_t size);

    static void setBaud(uint32_t baud, xpcc::IODevice* device);
    static enum flow_control get_flow_control(xpcc::IODevice* device);

private:
    bool _blocking_writes;
    xpcc::IODevice* _device;
};

#endif // __AP_HAL_EMPTY_UARTDRIVER_H__
