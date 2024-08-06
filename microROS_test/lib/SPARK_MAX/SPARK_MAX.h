#ifndef SPARK_MAX_H
#define SPARK_MAX_H

#include <Arduino.h>
#include <mcp_can.h>

#include "Comet_CAN_Common.h"
#include "CAN_Device_Interface.h"
#include "Comet_CAN_Helper.h"

/*********************************************************************************************************
** SPARK_MAX class
** For configuring and controlling SPARK MAXs
*********************************************************************************************************/
class SPARK_MAX : public ICAN_Device {
public:
    /*
    * Constructor
    */
    SPARK_MAX(uint8_t device_id, MCP_CAN &CAN0, Comet_CAN_Helper &CAN_Helper) : CAN0(CAN0) {
        this->device_id = device_id;
        current_mode = control_mode::NONE;

        current_control_frame.dlc = CONTROL_DLC;
        current_control_frame.ext = 1;
        CAN_Helper.add_to_CAN_dev_arr(this);
    }

    /*
    * Functions
    */
    bool is_FRC() const override {
        return true; // Example implementation
    }

    uint8_t get_device_id() const override {
        if (is_FRC()) {
            return device_id;
        } else {
            return 0; // Return a default value or handle appropriately if ID cannot be reported
        }
    }

    can_frame get_current_frame() const override {
        return current_control_frame; 
    }

    void clear_current_frame() override {
        current_control_frame = empty_frame;
        current_control_frame.dlc = CONTROL_DLC;
        current_control_frame.ext = 1;
    }

    uint8_t send_control_frame(const control_mode mode, const float setpoint); // Command SPARK MAX output
    uint8_t set_status_frame_period(const status_frame_id frame, const uint16_t period); // Set period for SPARK MAX status frames

    // Default destructor
    ~SPARK_MAX() override = default;

private:
    /*
    * Constants/variables
    */
    uint8_t device_id;
    control_mode current_mode;
    can_frame current_control_frame = empty_frame; // Used to store the most recent control frame
    MCP_CAN &CAN0; // Change to set all the periodic status's at the constructor so it doesn't need to be stored

    /*
    * Control Frame
    */
    static constexpr uint8_t CONTROL_DLC = 8;
    static constexpr uint8_t CONTROL_WRITE_SIZE = 4; // Size (bytes) of actual data written into the Data Window

    /*
    * Status Frame
    */
    static constexpr uint8_t STATUS_DLC = 2;
    static constexpr uint8_t STATUS_WRITE_SIZE = 2; // Size (bytes) of actual data written into the Data Window
};

#endif
