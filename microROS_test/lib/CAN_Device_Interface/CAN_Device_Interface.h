#ifndef CAN_DEVICE_INTERFACE_H
#define CAN_DEVICE_INTERFACE_H

#include <Arduino.h>
#include "Comet_CAN_Common.h"

// Define the interface
class ICAN_Device {
public:

    virtual bool is_FRC() const = 0;

    virtual uint8_t get_device_id() const = 0;

    virtual can_frame get_current_frame() const = 0;

    virtual void clear_current_frame() = 0; // can't be constant since it modifies a member variable

    // Virtual destructor for proper cleanup of derived classes
    // Is this necessary?
    virtual ~ICAN_Device() {}
};

#endif