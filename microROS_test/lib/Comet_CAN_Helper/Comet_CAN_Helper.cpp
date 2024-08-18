#include "Comet_CAN_Helper.h"
#include <mcp_can.h>

/*********************************************************************************************************
** Function name:           data_to_float_32_bit
** Descriptions:            Converts four bytes (little-endian) to a IEEE floating point number
*********************************************************************************************************/
float Comet_CAN_Helper::data_to_float_32_bit(uint8_t * data, uint8_t size){
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
** Function name:           parse_volt_and_amp
** Descriptions:            gets the float value for the voltage and currrent of the SPARK MAX
*********************************************************************************************************/
void Comet_CAN_Helper::parse_volt_and_amp(float * voltage, float * current, uint8_t *data){
  uint16_t voltage = ((uint16_t)data[6] << 8 & 0b00001111) | data[5]; // need to graph to find constant for converting into a float
  uint16_t current = ((uint16_t)data[7] << 4) | (data[6] & 0b11110000); // need to graph to find constant for converting into a float
}

/*********************************************************************************************************
** Function name:           parse_CAN_frame
** Descriptions:            Handles how to parse different CAN frames
*********************************************************************************************************/
void Comet_CAN_Helper::parse_CAN_frame(){
  if(!digitalRead(CAN0_INT)) { // If CAN0_INT pin is low, read receive buffer
      CAN0.readMsgBuf(&rxId, &len, rxBuf);
      
      // Handle data parsing for specific frames
      if ((rxId & FRC_dev_id_mask) == status_0) {
        parse_status_frame_0(rxBuf);
      } else if ((rxId & FRC_dev_id_mask) == status_1) {
        parse_status_frame_1(rxBuf, len);
      } else if ((rxId & FRC_dev_id_mask) == status_2) {
        parse_status_frame_2(rxBuf, len);
      }
      // Add more cases if necessary
    }
}

/*********************************************************************************************************
** Function name:           parse_status_frame_0
** Descriptions:            Function to parse SPARK MAX Periodic Status Frame 0
*********************************************************************************************************/
void Comet_CAN_Helper::parse_status_frame_0(uint8_t * data){
  float applied_output = ((static_cast<int>(data[1]) << 8) | data[0]) / 32764.0; // Range = [-1, 1]
}

/*********************************************************************************************************
** Function name:           parse_status_frame_1
** Descriptions:            Function to parse SPARK MAX Periodic Status Frame 1
*********************************************************************************************************/
void Comet_CAN_Helper::parse_status_frame_1(uint8_t * data, uint8_t size){
  float velocity = data_to_float_32_bit(data, size); // RPM
  uint8_t temperature = data[5]; // Celcius
  float voltage = 0.0;
  float current = 0.0;
}

/*********************************************************************************************************
** Function name:           parse_status_frame_2
** Descriptions:            Function to parse SPARK MAX Periodic Status Frame 2
*********************************************************************************************************/
void Comet_CAN_Helper::parse_status_frame_2(uint8_t * data, uint8_t size){
  float position = data_to_float_32_bit(data, size); // Rotations
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
byte Comet_CAN_Helper::add_to_CAN_dev_arr(ICAN_Device *CAN_dev){
  if (CAN_dev->is_FRC()){
    uint8_t device_id = CAN_dev->get_device_id();

    // Check to Ensure Unique Device ID for FRC devices
    for (int i = 0; i < MAX_CAN_DEVICES; i++){
      if (can_devices[i] != nullptr){
        if (can_devices[i]->get_device_id() == device_id){
          return CAN_FAIL; // ID in Use
        }
      }
    }
  }

  // Add the CAN_dev object to the array
    if (num_CAN_devs < MAX_CAN_DEVICES){
        can_devices[num_CAN_devs] = CAN_dev;
        num_CAN_devs++;
        return CAN_OK; // Success
    } else {
        return CAN_FAIL; // Array is full
    }
}

/*********************************************************************************************************
** Function name:           send_message
** Descriptions:            Returns the queued CAN frame of the selected CAN dev and then moves to the next dev
*********************************************************************************************************/
String Comet_CAN_Helper::send_message(){
  if (num_CAN_devs == 0){
    return "Sent nothing";
  }

  // Loopback to first CAN device
  if (selected_CAN_dev == num_CAN_devs){
    selected_CAN_dev = 0;
  }


  if (can_devices[selected_CAN_dev]->is_active()){
    can_frame frame = can_devices[selected_CAN_dev]->get_current_frame();

    // Check for empty frame since only empty frames should have a CAN ID (not FRC device ID, there's a difference) of 0
    if (frame.can_id == 0){
      return "Sent nothing" + String(selected_CAN_dev++);
    }


    CAN0.sendMsgBuf(frame.can_id, frame.ext, frame.dlc, frame.buf);

    // May want to either keep most recent command or dump it, not sure yet
    can_devices[selected_CAN_dev]->clear_current_frame();

    return "Sent something" + String(selected_CAN_dev ++);
  }
  else{
    int16_t next_device = get_next_enabled_device();
    
    if (next_device != NO_ACTIVE_CAN_DEVICES){
      selected_CAN_dev = next_device;
      return send_message();
    }
    else{
      return "Sent nothing everything DEACTIVATED";
    }
  }


  
}

/*********************************************************************************************************
** Function name:           get_next_enabled_device
** Descriptions:            returns -1 if no enabled devices, else returns next devices (increasing).
*********************************************************************************************************/
int16_t Comet_CAN_Helper::get_next_enabled_device(){
  for (int i = selected_CAN_dev + 1; i < selected_CAN_dev + num_CAN_devs; i++){
    if (can_devices[i % num_CAN_devs]->is_active()){
      return i % num_CAN_devs;
    }
  }
  return NO_ACTIVE_CAN_DEVICES;
}





