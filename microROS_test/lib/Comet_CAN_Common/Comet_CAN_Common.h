#ifndef COMET_CAN_COMMON_H
#define COMET_CAN_COMMON_H

#include <Arduino.h>

#define MAX_CAN_DEVICES 64 // FRC protocol limits number of devices to 64 so it's safe to use

static const uint32_t FRC_dev_id_mask = 0xFFFFFFC0; // Mask everything but device ID bits (last 6). Follows FRC CAN Protocol

/*
* CAN Frame structure
*/
struct can_frame {
    uint32_t can_id; // CAN identifier
    uint8_t ext;     // Extended frame flag
    uint8_t dlc;     // Data length code
    uint8_t buf[8];  // Data buffer
};

static can_frame empty_frame = {0, 0, 0, {0}}; // Set all members to 0

/*
* SPARK MAX periodic status frames
*/
enum status_frame_id {
    status_0 = 0x2051800,
    status_1 = 0x2051840,
    status_2 = 0x2051880,
    status_3 = 0x20518C0,
    status_4 = 0x2051900
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

/*********************************************************************************************************
** Function name:           create_data
** Descriptions:            Copy data to frame_data (little-Endian)
                                Meant for packing bytes into CAN frame
*********************************************************************************************************/
inline void create_data(byte *frame_data, const void *data,  const uint8_t write_size, const uint8_t dlc) {
    const byte *data_arr = static_cast<const byte *>(data);
    for (uint8_t i = 0; i < write_size; ++i) {
        frame_data[i] = data_arr[i];
    }

    // Fill remaining space with zeros
    for (uint8_t i = write_size; i < dlc; ++i) {
        frame_data[i] = 0;
    }
}

#endif // COMET_CAN_COMMON_H
