/*
::::To Do::::
++++ DONE
---- TODO
~~~~ IDEA

SPARK MAX Class
++++ update send_control_frame to check and set current_mode. Either use commanded mode or current mode depending on current mode status
++++ check set_status_frame_period function implementation
---- Built in ROS publisher for motor data????
      should be handled individually
---- Implement can_frame_queue
    ~~~~ Could implement the can_frame_queue in two ways. 
        1. Use a stack and add/remove frames to it from functions
            commander sends first frame sandwhiched bewtween hearbeats
        2. Use an array of CAN_devices. where each can_device updates it's desired command
            commander cycles through devices and send desired command sandwhiched between heartbeats
*/

#ifndef COMET_CAN_HELPER_H
#define COMET_CAN_HELPER_H
#define MAX_CAN_DEVICES 64 // FRC protocol limits number of devices to 64 so it's safe to use

#include <Arduino.h>
#include <mcp_can.h>
#include "Comet_CAN_Common.h"



/*********************************************************************************************************
** Comet_CAN_Helper class
** For handling the MCP2515, both reading and writing
*********************************************************************************************************/
class Comet_CAN_Helper {
public:
    /*
    * Constructor
    */
    Comet_CAN_Helper(MCP_CAN &CAN0) : CAN0(CAN0) {
        // Initialize the array of pointers to nullptr
        for (int i = 0; i < MAX_CAN_DEVICES; ++i) {
            Spark_Max_arr[i] = nullptr;
        }
    }

    /*
    * Constants/variables
    */
    const uint32_t DEVICE_ID_MASK = 0xFFFFFFC0; // Mask everything but device ID bits (last 6). Follows FRC CAN Protocol
    SPARK_MAX* Spark_Max_arr[MAX_CAN_DEVICES]; // Array of pointers to SPARK_MAX objects
    uint8_t num_Spark_Maxs = 0;

    

    /*
    * Functions
    */
    void parse_status_frame_0(uint8_t *data);                 // Parse status frame 0
    void parse_status_frame_1(uint8_t *data, uint8_t size);   // Parse status frame 1
    void parse_status_frame_2(uint8_t *data, uint8_t size);   // Parse status frame 2
    uint8_t send_enabled_heartbeat();             // Send enabled heartbeat
    uint8_t send_disabled_heartbeat();            // Send disabled heartbeat
    int add_to_Spark_Max_arr(SPARK_MAX *Spark_Max);        // Adds the Spark Max object to the list of Spark Maxs

private:
    /*
    * Constants/variables
    */
    MCP_CAN &CAN0;


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