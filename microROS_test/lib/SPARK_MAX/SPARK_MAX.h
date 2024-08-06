#ifndef SPARK_MAX_H
#define SPARK_MAX_H

#include <Arduino.h>
#include <mcp_can.h>

#include "Comet_CAN_Common.h"
#include "CAN_Device_Interface.h"

/*********************************************************************************************************
** SPARK_MAX class
** For configuring and controlling SPARK MAXs
*********************************************************************************************************/
class SPARK_MAX : public ICAN_Device  {
public:
    /*
    * Constructor
    */
    SPARK_MAX(uint8_t device_id, MCP_CAN &CAN0) : CAN0(CAN0) {
        this->device_id = device_id;
        current_mode = control_mode::NONE;

        current_control_frame.dlc = CONTROL_DLC;
        current_control_frame.ext = 1;
    }

    
    /*
    * Functions
    */
    bool is_FRC() const override {
        return true; // Example implementation
    }

    uint8_t get_device_id() const override {
        return device_id;
    }

    can_frame get_current_frame() const override {
        return current_control_frame;
    }

    uint8_t send_control_frame(const control_mode mode, // Command SPARK MAX output
                               const float setpoint);
    uint8_t set_status_frame_period(const status_frame_id frame, // Set period for SPARK MAX status frames
                                    const uint16_t period);

    // Default destructor
    ~SPARK_MAX() override = default;
    

private:
    /*
    * Constants/variables
    */
    uint8_t device_id;
    control_mode current_mode;
    can_frame current_control_frame; // Used to store the most recent control frame
    MCP_CAN &CAN0;


    /*
    * Control Frame
    */
    const uint8_t CONTROL_DLC = 8;
    const uint8_t CONTROL_WRITE_SIZE = 4; // Size (bytes) of actual data written into the Data Window

    /*
    * Status Frame
    */
    const uint8_t STATUS_DLC = 2;
    const uint8_t STATUS_WRITE_SIZE = 2; // Size (bytes) of actual data written into the Data Window
};

#endif