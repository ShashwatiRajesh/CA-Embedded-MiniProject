/*
::::To Do::::
++++ DONE
---- TODO

SPARK MAX Class
++++ update send_control_frame to check and set current_mode. Either use commanded mode or current mode depending on current mode status
---- check set_status_frame_period function implementation
*/

#ifndef COMET_CAN_HELPER_H
#define COMET_CAN_HELPER_H

#include <Arduino.h>
#include <mcp_can.h>

/*********************************************************************************************************
** Constants
*********************************************************************************************************/
enum status_frame_id {
    status_0 = 0x2051800,
    status_1 = 0x2051840,
    status_2 = 0x2051880,
    status_3 = 0x20518C0,
    status_4 = 0x2051900
};

/*********************************************************************************************************
** Comet_CAN_Helper class
** For handling the MCP2515, both reading and writing
*********************************************************************************************************/
class Comet_CAN_Helper {
public:
    /*
    * Constants/variables
    */
    const uint32_t DEVICE_ID_MASK = 0xFFFFFFC0; // Mask everything but device ID bits (last 6). Follows FRC CAN Protocol

    /*
    * CAN Frame structure
    */
    struct can_frame {
        uint32_t can_id; // CAN identifier
        uint8_t ext;     // Extended frame flag
        uint8_t dlc;     // Data length code
        uint8_t buf[8];  // Data buffer
    };

    /*
    * Functions
    */
    void parse_status_frame_0(uint8_t *data);                 // Parse status frame 0
    void parse_status_frame_1(uint8_t *data, uint8_t size);   // Parse status frame 1
    void parse_status_frame_2(uint8_t *data, uint8_t size);   // Parse status frame 2
    uint8_t send_enabled_heartbeat(MCP_CAN CAN0);             // Send enabled heartbeat
    uint8_t send_disabled_heartbeat(MCP_CAN CAN0);            // Send disabled heartbeat

private:
    /*
    * Constants/variables
    */
    can_frame can_frame_queue[10];

    /*
    * Heartbeat frame
    */
    const uint32_t HEARTBEAT_ID = 0x2052C80;
    const uint8_t HEARTBEAT_DLC = 8;
    byte heartbeat_data_enabled[8] = {255, 255, 255, 255, 255, 255, 255, 255};
    byte heartbeat_data_disabled[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    /*
    * MCP2515 Masks
    */
    const uint32_t MASK0 = 0xFFFFFFC0; // Mask everything but device ID bits (last 6). Follows FRC CAN Protocol

    /*
    * MCP2515 Filters
    */
    const uint32_t PERIODIC_STATUS_0_FILTER = 0x82051800; // SPARK MAX periodic CAN frames
    const uint32_t PERIODIC_STATUS_1_FILTER = 0x82051840; // SPARK MAX periodic CAN frames
    const uint32_t PERIODIC_STATUS_2_FILTER = 0x82051880; // SPARK MAX periodic CAN frames
    const uint32_t EMPTY_FILTER = 0x00000000;             // Empty Filter  

    /*
    * Functions
    */
    float data_to_float(uint8_t *data, uint8_t size); // Converts four bytes (little-endian) to a IEEE floating point number
};

/*********************************************************************************************************
** SPARK_MAX class
** For configuring and controlling SPARK MAXs
*********************************************************************************************************/
class SPARK_MAX {
public:
    /*
    * Constructor
    */
    SPARK_MAX(uint8_t device_id) {
        this->device_id = device_id;
        current_mode = control_mode::NONE;
    };

    /*
    * SPARK MAX control modes
    */
    enum control_mode {
        Use_Current_Mode = 0x2050040,
        Duty_Cycle_Set = 0x2050080,
        Speed_Set = 0x2050480,
        Smart_Velocity_Set = 0x20504C0,
        Position_Set = 0x2050C80,
        Voltage_Set = 0x2051080,
        Current_Set = 0x20510C0,
        Smart_Motion_Set = 0x2051480,
        NONE = 0x00000000
    };

    /*
    * Functions
    */
    uint8_t send_control_frame(MCP_CAN CAN0,            // Command SPARK MAX output
                               const uint32_t device_id,
                               const control_mode mode, 
                               const float setpoint);
    uint8_t set_status_frame_period(MCP_CAN CAN0,       // Set period for SPARK MAX status frames
                                    const uint32_t device_id,
                                    const status_frame_id frame, 
                                    const uint16_t period);

private:
    /*
    * Constants/variables
    */
    uint8_t device_id;
    control_mode current_mode;

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

    /*
    * Functions
    */
    void create_data(const void *data,                 // Copy data to frame_data (little-Endian)
                     byte *frame_data, 
                     const uint8_t write_size, 
                     const uint8_t dlc);
};

#endif
