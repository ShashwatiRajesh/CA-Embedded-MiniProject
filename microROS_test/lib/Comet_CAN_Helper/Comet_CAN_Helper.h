#ifndef COMET_CAN_HELPER_H
#define COMET_CAN_HELPER_H

#include <Arduino.h>

/*********************************************************************************************************
** Comet_CAN_Helper class
*********************************************************************************************************/
class Comet_CAN_Helper{
public:

/*
* SPARK MAX control modes
*/
enum control_mode {
  Duty_Cycle_Set = 0x2050080,
  Speed_Set = 0x2050480,
  Smart_Velocity_Set = 0x20504C0,
  Position_Set = 0x2050C80,
  Voltage_Set = 0x2051080,
  Current_Set = 0x20510C0,
  Smart_Motion_Set = 0x2051480
};

/*
* SPARK MAX status frame IDs
*/
enum status_frame_id {
  status_0 = 0x2051800,
  status_1 = 0x2051840,
  status_2 = 0x2051880,
  status_3 = 0x20518C0,
  status_4 = 0x2051900
};

/*
* Functions
*/
float data_to_float(uint8_t * data, uint8_t size);                      // Converts four bytes (little-endian) to a IEEE floating point number
void parse_status_frame_0(uint8_t * data);                              // Function to parse SPARK MAX Periodic Status Frame 0
void parse_status_frame_1(uint8_t * data, uint8_t size);                // Function to parse SPARK MAX Periodic Status Frame 1
void parse_status_frame_2(uint8_t * data, uint8_t size);                // Function to parse SPARK MAX Periodic Status Frame 2


private:
    // Private members and methods

};

/*********************************************************************************************************
** Constants
*********************************************************************************************************/
/*
* Heartbeat frame
*/
const uint32_t HEARTBEAT_ID = 0x2052C80;
const uint8_t HEARTBEAT_DLC = 8;

/*
* MCP2515 Masks
*/
const uint32_t DEVICE_ID_MASK = 0xFFFFFFC0;                             // Mask everything but device ID bits (last 6). Follows FRC CAN Protocol

/*
* MCP2515 Filters
*/
const uint32_t PERIODIC_STATUS_0_FILTER = 0x82051800;                   // SPARK MAX periodic CAN frames
const uint32_t PERIODIC_STATUS_1_FILTER = 0x82051840;                   // SPARK MAX periodic CAN frames
const uint32_t PERIODIC_STATUS_2_FILTER = 0x82051880;                   // SPARK MAX periodic CAN frames
const uint32_t EMPTY_FILTER = 0x00000000;                               // Empty Filter

#endif