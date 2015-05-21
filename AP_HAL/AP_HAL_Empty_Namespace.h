
#ifndef __AP_HAL_EMPTY_NAMESPACE_H__
#define __AP_HAL_EMPTY_NAMESPACE_H__

/* While not strictly required, names inside the Empty namespace are prefixed
 * with Empty for clarity. (Some of our users aren't familiar with all of the
 * C++ namespace rules.)
 */

namespace XpccHAL {
    class UARTDriver;
    class I2CDriver;
    class SPIDeviceManager;
    class SPIDeviceDriver;
    class AnalogSource;
    class AnalogIn;
    class Storage;
    class GPIO;
    class DigitalSource;
    class RCInput;
    class RCOutput;
    class Semaphore;
    class Scheduler;
    class Util;
    class EmptyPrivateMember;
}

#endif // __AP_HAL_EMPTY_NAMESPACE_H__

