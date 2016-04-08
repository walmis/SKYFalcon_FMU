
#ifndef __AP_HAL_EMPTY_CLASS_H__
#define __AP_HAL_EMPTY_CLASS_H__

#include <AP_HAL/AP_HAL.h>

#include "AP_HAL_Empty_Namespace.h"
#include "PrivateMember.h"

class HAL_XPCC final : public AP_HAL::HAL {
public:
    HAL_XPCC();

    void run(int argc, char * const argv[], Callbacks* callbacks) const override;

};

extern const HAL_XPCC AP_HAL_XPCC;

#endif // __AP_HAL_EMPTY_CLASS_H__

