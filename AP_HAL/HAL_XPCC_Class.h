
#ifndef __AP_HAL_EMPTY_CLASS_H__
#define __AP_HAL_EMPTY_CLASS_H__

#include <AP_HAL.h>

#include "AP_HAL_Empty_Namespace.h"
#include "PrivateMember.h"

class HAL_XPCC final : public AP_HAL::HAL {
public:
    HAL_XPCC();
    void init(int argc, char * const * argv) const;
};

extern const HAL_XPCC AP_HAL_XPCC;

#endif // __AP_HAL_EMPTY_CLASS_H__

