#include "PDP.h"
#include <Comet_CAN_Helper.h>


/*********************************************************************************************************
** Function name:           parse_status_frame_1
** Descriptions:            Function to parse PDP Periodic Status Frame 1
*********************************************************************************************************/
void PDP::parse_status_frame_1(uint8_t * data, uint8_t size){
  for (int i = 0; i <= 5; i++){
    status.currents[i] = data[i] * kCurrentScalar;
  }
}

/*********************************************************************************************************
** Function name:           parse_status_frame_2
** Descriptions:            Function to parse PDP Periodic Status Frame 2
*********************************************************************************************************/
void PDP::parse_status_frame_2(uint8_t * data, uint8_t size){
  for (int i = 6; i <= 11; i++){
    status.currents[i] = data[i-6] * kCurrentScalar;
  }
}

/*********************************************************************************************************
** Function name:           parse_status_frame_3
** Descriptions:            Function to parse PDP Periodic Status Frame 3
*********************************************************************************************************/
void PDP::parse_status_frame_3(uint8_t * data, uint8_t size){
  for (int i = 12; i <= 15; i++){
    status.currents[i] = data[i-12] * kCurrentScalar;
  }
  uint8_t vbatByte = data[5]; /* byte 6 */
  /* scale it to volts */
  float volts = 0.05f * vbatByte + 4.0f;
  status.voltage = volts;

  status.temperature = data[7] * 1.03250836957542 - 67.8564500484966; // byte 8????
}
