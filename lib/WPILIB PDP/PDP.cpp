#include "PDP.h"
#include <Comet_CAN_Helper.h>


/*********************************************************************************************************
** Function name:           parse_status_frame_1
** Descriptions:            Function to parse PDP Periodic Status Frame 1
*********************************************************************************************************/
void PDP::parse_status_frame_1(uint8_t * data, uint8_t size){
  //if you're reading this i'm so sorry
  status.currents[0] = (static_cast<uint32_t>(data[0]) << 2) | (data[1] & ((1 << 2) - 1));
  //no, it doesn't get better
  status.currents[1] = (static_cast<uint32_t>(data[1] >> (8-6)) << 4) | (data[2] & ((1 << 4) - 1));
  status.currents[2] = (static_cast<uint32_t>(data[2] >> (8-4)) << 6) | (data[3] & ((1 << 6) - 1));
  status.currents[3] = (static_cast<uint32_t>(data[3] >> (8-2)) << 6) | data[4];
  status.currents[4] = (static_cast<uint32_t>(data[5]) << 2) | (data[6] & ((1 << 2) - 1));
  status.currents[5] = (static_cast<uint32_t>(data[6] >> (8-6)) << 4) | (data[7] & ((1 << 4) - 1));
}

/*********************************************************************************************************
** Function name:           parse_status_frame_2
** Descriptions:            Function to parse PDP Periodic Status Frame 2
*********************************************************************************************************/
void PDP::parse_status_frame_2(uint8_t * data, uint8_t size){
  //if you're reading this i'm so sorry
  status.currents[6] = (static_cast<uint32_t>(data[0]) << 2) | (data[1] & ((1 << 2) - 1));
  //no, it doesn't get better
  status.currents[7] = (static_cast<uint32_t>(data[1] >> (8-6)) << 4) | (data[2] & ((1 << 4) - 1));
  status.currents[8] = (static_cast<uint32_t>(data[2] >> (8-4)) << 6) | (data[3] & ((1 << 6) - 1));
  status.currents[9] = (static_cast<uint32_t>(data[3] >> (8-2)) << 6) | data[4];
  status.currents[10] = (static_cast<uint32_t>(data[5]) << 2) | (data[6] & ((1 << 2) - 1));
  status.currents[11] = (static_cast<uint32_t>(data[6] >> (8-6)) << 4) | (data[7] & ((1 << 4) - 1));
}

/*********************************************************************************************************
** Function name:           parse_status_frame_3
** Descriptions:            Function to parse PDP Periodic Status Frame 3
*********************************************************************************************************/
void PDP::parse_status_frame_3(uint8_t * data, uint8_t size){
  status.currents[12] = (static_cast<uint32_t>(data[0]) << 2) | (data[1] & ((1 << 2) - 1));
  //no, it doesn't get better
  status.currents[13] = (static_cast<uint32_t>(data[1] >> (8-6)) << 4) | (data[2] & ((1 << 4) - 1));
  status.currents[14] = (static_cast<uint32_t>(data[2] >> (8-4)) << 6) | (data[3] & ((1 << 6) - 1));
  status.currents[15] = (static_cast<uint32_t>(data[3] >> (8-2)) << 6) | data[4];
  status.internalResBattery_mOhms = data[5];
  status.voltage = data[6];
  status.internalResBattery_mOhms = data[7];
}
