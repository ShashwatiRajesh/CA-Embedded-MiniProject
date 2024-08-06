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
** Function name:           send_disabled_heartbeat
** Descriptions:            Send heartbeat to disable all devices
*********************************************************************************************************/
bool Comet_CAN_Helper::add_frame_to_queue(const can_frame &frame) {
    if (queue_count == QUEUE_SIZE) {
        return false; // Queue is full
    }
    can_frame_queue[queue_rear] = frame;
    queue_rear = (queue_rear + 1) % QUEUE_SIZE;
    ++queue_count;
    return true;
}

/*********************************************************************************************************
** Function name:           send_disabled_heartbeat
** Descriptions:            Send heartbeat to disable all devices
*********************************************************************************************************/
bool Comet_CAN_Helper::remove_frame_from_queue(can_frame &frame) {
    if (is_queue_empty()) {
        return false; // Queue is empty
    }
    frame = can_frame_queue[queue_front];
    queue_front = (queue_front + 1) % QUEUE_SIZE;
    --queue_count;
    return true;
}

/*********************************************************************************************************
** Function name:           is_queue_empty
** Descriptions:            Returns if the CAN frame queue is empty
*********************************************************************************************************/
bool Comet_CAN_Helper::is_queue_empty() const {
    return queue_count == 0;
}

/*********************************************************************************************************
** Function name:           create_data
** Descriptions:            Copy data to frame_data (little-Endian)
*********************************************************************************************************/
void SPARK_MAX::create_data(const void *data, byte *frame_data, const uint8_t write_size, const uint8_t dlc){
  const byte *data_arr = static_cast<const byte *>(data);
  for (int i = 0; i < write_size; i++) {
    frame_data[i] = data_arr[i];
  }

  // Fill remaining space with zeros
  for (int i = write_size; i < dlc; i++) {
    frame_data[i] = 0;
  }
}

/*********************************************************************************************************
** Function name:           send_control_frame
** Descriptions:            Function to command SPARK MAX ouput
*********************************************************************************************************/
uint8_t SPARK_MAX::send_control_frame(const control_mode mode, const float setpoint){
  if (mode != control_mode::NONE){
    uint8_t frame_data[CONTROL_DLC];
    create_data(&setpoint, frame_data, CONTROL_WRITE_SIZE, CONTROL_DLC);

    if (current_mode != mode){
      current_mode = mode;
      if(CAN0.sendMsgBuf(device_id + mode, CAN_EXTID, CONTROL_DLC, frame_data) == CAN_OK){
        return CAN_OK;
      } 
      else {
        return CAN_FAIL;
      }
    }
    else{
      if(CAN0.sendMsgBuf(device_id + control_mode::Use_Current_Mode, CAN_EXTID, CONTROL_DLC, frame_data) == CAN_OK){
        return CAN_OK;
      } 
      else {
        return CAN_FAIL;
      }
    }
  }

  return -1;
}

/*********************************************************************************************************
** Function name:           set_status_frame_period
** Descriptions:            Function to set period for SPARK MAX status frames
*********************************************************************************************************/
uint8_t SPARK_MAX::set_status_frame_period(const status_frame_id frame, const uint16_t period){
  uint8_t frame_data[STATUS_DLC];
  create_data(&period, frame_data, STATUS_WRITE_SIZE, STATUS_DLC);
  if(CAN0.sendMsgBuf(device_id + frame, CAN_EXTID, STATUS_DLC, frame_data) == CAN_OK){
    return CAN_OK;
  } 
  else {
    return CAN_FAIL;
  }
}

