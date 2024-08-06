#include "Comet_CAN_Helper.h"
#include <mcp_can.h>

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

/*********************************************************************************************************
** Function name:           send_enabled_heartbeat
** Descriptions:            Send heartbeat to enable all devices
*********************************************************************************************************/
uint8_t Comet_CAN_Helper::send_enabled_heartbeat(){
  if(CAN0.sendMsgBuf(HEARTBEAT_ID, CAN_EXTID, HEARTBEAT_DLC, heartbeat_data_enabled) == CAN_OK){
      return CAN_OK;
    } 
    else {
      return CAN_FAIL;
    }
}

/*********************************************************************************************************
** Function name:           send_disabled_heartbeat
** Descriptions:            Send heartbeat to disable all devices
*********************************************************************************************************/
uint8_t Comet_CAN_Helper::send_disabled_heartbeat(){
  if(CAN0.sendMsgBuf(HEARTBEAT_ID, CAN_EXTID, HEARTBEAT_DLC, heartbeat_data_disabled) == CAN_OK){
      return CAN_OK;
    } 
    else {
      return CAN_FAIL;
    }
}

/*********************************************************************************************************
** Function name:           add_to_CAN_dev_arr
** Descriptions:            Adds a device to the CAN device list
*********************************************************************************************************/
int Comet_CAN_Helper::add_to_CAN_dev_arr(ICAN_Device *CAN_dev){
  if (CAN_dev->is_FRC()){
    uint8_t device_id = CAN_dev->get_device_id();

    // Check to Ensure Unique Device ID for FRC devices
    for (int i = 0; i < MAX_CAN_DEVICES; i++){
      if (can_devices[i] != nullptr){
        if (can_devices[i]->get_device_id() == device_id){
          return -1; // ID in Use
        }
      }
    }
  }

  // Add the CAN_dev object to the array
    if (num_CAN_devs < MAX_CAN_DEVICES){
        can_devices[num_CAN_devs] = CAN_dev;
        num_CAN_devs++;
        return 0; // Success
    } else {
        return -2; // Array is full
    }
}