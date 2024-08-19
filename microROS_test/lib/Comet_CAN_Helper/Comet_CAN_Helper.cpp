#include "Comet_CAN_Helper.h"
#include <mcp_can.h>

/*********************************************************************************************************
** Function name:           parse_CAN_frame
** Descriptions:            Handles how to parse different CAN frames
*********************************************************************************************************/
void Comet_CAN_Helper::parse_CAN_frame(){
  if(!digitalRead(CAN0_INT)) { // If CAN0_INT pin is low, read receive buffer
      CAN0.readMsgBuf(&rxId, &len, rxBuf);

      int device_index = get_device_from_FRC_id(rxId & (~FRC_dev_id_mask)); // Filter out everything but FRC device ID

      if (device_index != NO_MATCHING_FRC_DEVICE_ID){
        can_devices[device_index]->parse_CAN_frame(rxId, len, rxBuf);
      }
    }
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

/*********************************************************************************************************
** Function name:           get_device_from_FRC_id
** Descriptions:            returns the index in the CAN_dev_arr of the device with the given FRC_id
*********************************************************************************************************/
uint8_t Comet_CAN_Helper::get_device_from_FRC_id(uint8_t frc_id){
  for(int i = 0; i < num_CAN_devs; i++){
    if (can_devices[i]->get_device_id() == frc_id){
      return i;
    }
  }
  return NO_MATCHING_FRC_DEVICE_ID;
}




