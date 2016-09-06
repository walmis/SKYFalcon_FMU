
#ifndef __AP_HAL_EMPTY_CLASS_H__
#define __AP_HAL_EMPTY_CLASS_H__

#include <AP_HAL/AP_HAL.h>

#include "../HAL/AP_HAL_Empty_Namespace.h"
#include "AnalogIn.h"
#include "GPIO.h"
#include "RCInput.h"
#include "RCOutput.h"
#include "Util.h"
#include "Scheduler.h"
#include "Storage.h"
#include "UARTDriver.h"
#include "I2CDevice.h"

class HAL_XPCC final : public AP_HAL::HAL {
public:
    HAL_XPCC();

    void run(int argc, char * const argv[], Callbacks* callbacks) const override;
};


#endif // __AP_HAL_EMPTY_CLASS_H__

