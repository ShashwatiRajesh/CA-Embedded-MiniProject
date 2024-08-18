/*
::::To Do::::
++++ DONE
---- TODO
~~~~ IDEA

---- Define explicit error codes for adding devices to arr and other functions
*/

#ifndef COMET_CAN_HELPER_H
#define COMET_CAN_HELPER_H
#define NO_ACTIVE_CAN_DEVICES -1


#include <Arduino.h>
#include <mcp_can.h>
#include "Comet_CAN_Common.h"
#include "CAN_Device_Interface.h"

/*********************************************************************************************************
** Comet_CAN_Helper class
** For handling the MCP2515, both reading and writing
** Assume the only function sending CAN messages is the CAN_Core callback function
*********************************************************************************************************/
class Comet_CAN_Helper {
public:
    /*
    * Constructor
    */
    Comet_CAN_Helper(MCP_CAN &CAN0) : CAN0(CAN0) {
        // Initialize the array of pointers to nullptr
        for (int i = 0; i < MAX_CAN_DEVICES; ++i) {
            can_devices[i] = nullptr;
        }
    }

    /*
    * Functions
    */
    void parse_status_frame_0(uint8_t *data);                 // Parse status frame 0
    void parse_status_frame_1(uint8_t *data, uint8_t size);   // Parse status frame 1
    void parse_status_frame_2(uint8_t *data, uint8_t size);   // Parse status frame 2

    // CAN Core functions
    uint8_t send_enabled_heartbeat();             // Send enabled heartbeat
    uint8_t send_disabled_heartbeat();            // Send disabled heartbeat
    String send_message();   // Sends the queued CAN frame of the selected CAN dev and then moves to the next dev

    // SHOULD ONLY BE CALLED IN SETUP() OR DURING DEVICE INITIALIZATION
    byte add_to_CAN_dev_arr(ICAN_Device *CAN_dev);        // Adds the Spark Max object to the list of Spark Maxs
    int16_t get_next_enabled_device();
    

private:
    /*
    * Constants/variables
    */
    MCP_CAN &CAN0;
    ICAN_Device* can_devices[MAX_CAN_DEVICES]; // Array of pointers to CAN devices
    uint8_t num_CAN_devs = 0;
    uint32_t selected_CAN_dev = 0;
    const uint32_t DEVICE_ID_MASK = FRC_dev_id_mask; // Mask everything but device ID bits (last 6). Follows FRC CAN Protocol


    /*
    * Heartbeat frame
    */
    const uint32_t HEARTBEAT_ID = 0x2052C80;
    const uint8_t HEARTBEAT_DLC = 8;
    uint8_t heartbeat_data_enabled[8] = {255, 255, 255, 255, 255, 255, 255, 255};
    uint8_t heartbeat_data_disabled[8] = {0, 0, 0, 0, 0, 0, 0, 0};

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

#endif