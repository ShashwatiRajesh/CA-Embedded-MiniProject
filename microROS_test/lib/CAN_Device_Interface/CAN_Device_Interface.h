#ifndef CAN_DEVICE_INTERFACE_H
#define CAN_DEVICE_INTERFACE_H

#include <Arduino.h>
#include "Comet_CAN_Common.h"

// Define the interface
class ICAN_Device {
public:

    virtual bool is_FRC() const = 0; // TRUE for FRC devices like SPARK MAX, FALSE for all others

    virtual uint8_t get_device_id() const = 0; // Specific to FRC devices, returns FRC device_id

    virtual can_frame get_current_frame() const = 0; // Returns current desired CAN frame/instruction

    virtual void clear_current_frame() = 0; // Clean up after a frame has been sent to the MCP2515 buffer

    virtual bool is_active()const = 0; // Returns the active state of a CAN device

    virtual void set_active(bool input) = 0; // Set's the active state of a CAN device

    // Does not include a set_current_frame function since different CAN devices will required different parameters to generate a CAN frame.
        // There may also be multiple possible CAN frames 

    // Destructor, probably not needed
    virtual ~ICAN_Device() {}
};

#endif