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

/*
* SPARK MAX status data structure
*/
struct SPARK_MAX_status{
    float applied_output;
    float velocity;
    float position;
    uint8_t temperature;
    float voltage;
    float current;
};

/*
* PDP status data structure
*/
struct PDP_status{
    float temperature;
    float voltage;
    u_int32_t currents[16];
};
static PDP_status empty_pdp_status = {0.0, 0.0, {}};
static SPARK_MAX_status empty_spark_max_status = {0, 0.0, 0.0, 0, 0.0, 0.0};

static can_frame empty_frame = {0, 0, 0, {0}}; // Set all members to 0

/*
* SPARK MAX periodic status frames
*/
enum SPARK_MAX_status_frame_id {
    status_0 = 0x2051800,
    status_1 = 0x2051840,
    status_2 = 0x2051880,
    status_3 = 0x20518C0,
    status_4 = 0x2051900
};

enum PDP_status_frame_id {
    STATUS_1 = 0x08041400, //Channels 0-5
    STATUS_2 = 0x08041440, //Channels 6-11
    STATUS_3 = 0x08041480 //Channels 12-15
};

enum PDP_status_frame_id2 {
    Status1 = 0x50, 
    Status2 = 0x51, 
    Status3 = 0x52,
    StatusEnergy = 0x5D,
    Control1 = 0x70
};
/* encoder/decoders */
union PdpStatus1 {
  uint8_t data[8];
  struct Bits {
    unsigned chan1_h8 : 8;
    unsigned chan2_h6 : 6;
    unsigned chan1_l2 : 2;
    unsigned chan3_h4 : 4;
    unsigned chan2_l4 : 4;
    unsigned chan4_h2 : 2;
    unsigned chan3_l6 : 6;
    unsigned chan4_l8 : 8;
    unsigned chan5_h8 : 8;
    unsigned chan6_h6 : 6;
    unsigned chan5_l2 : 2;
    unsigned reserved4 : 4;
    unsigned chan6_l4 : 4;
  } bits;
};

union PdpStatus2 {
  uint8_t data[8];
  struct Bits {
    unsigned chan7_h8 : 8;
    unsigned chan8_h6 : 6;
    unsigned chan7_l2 : 2;
    unsigned chan9_h4 : 4;
    unsigned chan8_l4 : 4;
    unsigned chan10_h2 : 2;
    unsigned chan9_l6 : 6;
    unsigned chan10_l8 : 8;
    unsigned chan11_h8 : 8;
    unsigned chan12_h6 : 6;
    unsigned chan11_l2 : 2;
    unsigned reserved4 : 4;
    unsigned chan12_l4 : 4;
  } bits;
};

union PdpStatus3 {
  uint8_t data[8];
  struct Bits {
    unsigned chan13_h8 : 8;
    unsigned chan14_h6 : 6;
    unsigned chan13_l2 : 2;
    unsigned chan15_h4 : 4;
    unsigned chan14_l4 : 4;
    unsigned chan16_h2 : 2;
    unsigned chan15_l6 : 6;
    unsigned chan16_l8 : 8;
    unsigned internalResBattery_mOhms : 8;
    unsigned busVoltage : 8;
    unsigned temp : 8;
  } bits;
};

union PdpStatusEnergy {
  uint8_t data[8];
  struct Bits {
    unsigned TmeasMs_likelywillbe20ms_ : 8;
    unsigned TotalCurrent_125mAperunit_h8 : 8;
    unsigned Power_125mWperunit_h4 : 4;
    unsigned TotalCurrent_125mAperunit_l4 : 4;
    unsigned Power_125mWperunit_m8 : 8;
    unsigned Energy_125mWPerUnitXTmeas_h4 : 4;
    unsigned Power_125mWperunit_l4 : 4;
    unsigned Energy_125mWPerUnitXTmeas_mh8 : 8;
    unsigned Energy_125mWPerUnitXTmeas_ml8 : 8;
    unsigned Energy_125mWPerUnitXTmeas_l8 : 8;
  } bits;
};
//disgusting if true
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
