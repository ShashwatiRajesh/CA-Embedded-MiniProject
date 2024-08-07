#include "SPARK_MAX.h"
#include <Comet_CAN_Helper.h>



/*********************************************************************************************************
** Function name:           set_control_frame
** Descriptions:            Function to command SPARK MAX ouput
*********************************************************************************************************/
uint8_t SPARK_MAX::set_control_frame(const control_mode mode, const float setpoint){
  if (mode != control_mode::NONE){
    xSemaphoreTake(mutex, portMAX_DELAY);
    // Double check to make sure DLC and ext are set
    current_control_frame.dlc = CONTROL_DLC;
    current_control_frame.ext = 1;

    uint8_t frame_data[CONTROL_DLC];
    create_data(&setpoint, frame_data, CONTROL_WRITE_SIZE, CONTROL_DLC);

    // To avoid repeatedly sending control type frames
    if (current_mode != mode){
        current_mode = mode;
        current_control_frame.can_id = device_id + mode;
        memcpy(current_control_frame.buf, frame_data, CONTROL_DLC);
    }
    else{
        current_control_frame.can_id = device_id + control_mode::Use_Current_Mode;
        memcpy(current_control_frame.buf, frame_data, CONTROL_DLC);
    }
    xSemaphoreGive(mutex);
  }

  return -1;
}

/*********************************************************************************************************
** Function name:           set_status_frame_period
** Descriptions:            Function to set period for SPARK MAX status frames
*********************************************************************************************************/
uint8_t SPARK_MAX::set_status_frame_period(const status_frame_id frame, const uint16_t period, MCP_CAN &CAN0){
  // Delay to ensure there is an available transmit buffer
  delay(50);
  uint8_t frame_data[STATUS_DLC];
  create_data(&period, frame_data, STATUS_WRITE_SIZE, STATUS_DLC);
  if(CAN0.sendMsgBuf(device_id + frame, CAN_EXTID, STATUS_DLC, frame_data) == CAN_OK){
    return CAN_OK;
  } 
  else {
    return CAN_FAIL;
  }
}

/*********************************************************************************************************
** Function name:           set_all_status_frame_periods
** Descriptions:            Function to set all periods for SPARK MAX status frames 0->4
*********************************************************************************************************/
void SPARK_MAX::set_all_status_frame_periods(MCP_CAN &CAN0, u_int16_t period0, u_int16_t period1, u_int16_t period2, u_int16_t period3, u_int16_t period4){
  set_status_frame_period(status_0, period0, CAN0);
  set_status_frame_period(status_1, period1, CAN0);
  set_status_frame_period(status_2, period2, CAN0);
  set_status_frame_period(status_3, period3, CAN0);
  set_status_frame_period(status_4, period4, CAN0);
}

