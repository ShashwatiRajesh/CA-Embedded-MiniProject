#include "Comet_CAN_Helper.h"

/*********************************************************************************************************
** Function name:           data_to_float
** Descriptions:            Converts four bytes (little-endian) to a IEEE floating point number
*********************************************************************************************************/
float Comet_CAN_Helper::data_to_float(uint8_t * data, uint8_t size){
  if (size >= 4 && data != nullptr){
    uint32_t intValue = ((uint32_t)data[3] << 24) |
                        ((uint32_t)data[2] << 16) |
                        ((uint32_t)data[1] << 8) |
                        data[0];

    float result;
    memcpy(&result, &intValue, sizeof(float));

    return result;
  }
  else{
    Serial.println("FAILED TO CONVERT TO FLOAT");
    return NAN;
  }
}

/*********************************************************************************************************
** Function name:           parse_status_frame_0
** Descriptions:            Function to parse SPARK MAX Periodic Status Frame 0
*********************************************************************************************************/
void Comet_CAN_Helper::parse_status_frame_0(uint8_t * data){
  Serial.print("Applied output : ");
  Serial.println(((static_cast<int>(data[1]) << 8) | data[0]) / 32764.0); // Range = [-1, 1]
}

/*********************************************************************************************************
** Function name:           parse_status_frame_1
** Descriptions:            Function to parse SPARK MAX Periodic Status Frame 1
*********************************************************************************************************/
void Comet_CAN_Helper::parse_status_frame_1(uint8_t * data, uint8_t size){
  Serial.print("Velocity : ");
  Serial.println(data_to_float(data, size)); // RPM
}

/*********************************************************************************************************
** Function name:           parse_status_frame_2
** Descriptions:            Function to parse SPARK MAX Periodic Status Frame 2
*********************************************************************************************************/
void Comet_CAN_Helper::parse_status_frame_2(uint8_t * data, uint8_t size){
  Serial.print("Position : ");
  Serial.println(data_to_float(data, size)); // Rotations
}