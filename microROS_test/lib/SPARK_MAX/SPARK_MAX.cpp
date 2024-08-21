#include "SPARK_MAX.h"
#include <Comet_CAN_Helper.h>

/*********************************************************************************************************
** Function name:           set_control_frame
** Descriptions:            Function to command SPARK MAX ouput
*********************************************************************************************************/
uint8_t SPARK_MAX::set_control_frame(const control_mode mode, const float setpoint){
  if (mode != control_mode::NONE){
    // Double check to make sure DLC and ext are set
    current_control_frame.dlc = CONTROL_DLC;
    current_control_frame.ext = 1;

    uint8_t frame_data[CONTROL_DLC];
    create_data(frame_data, &setpoint, CONTROL_WRITE_SIZE, CONTROL_DLC);

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
  }

  return -1;
}

/*********************************************************************************************************
** Function name:           update_status_0
** Descriptions:            Updates the applied ouput
*********************************************************************************************************/
void SPARK_MAX::update_status_0(float applied_output){
  status.applied_output = applied_output;
}

/*********************************************************************************************************
** Function name:           update_status_1
** Descriptions:            Updates the Velocity, Temperature, Voltage, and Current
*********************************************************************************************************/
void SPARK_MAX::update_status_1(float velocity, float temperature, float voltage, float current){
  status.velocity = velocity;
  status.temperature = temperature;
  status.voltage = voltage;
  status.current = current;
}
    
/*********************************************************************************************************
** Function name:           update_status_2
** Descriptions:            Updates the Position
*********************************************************************************************************/
void SPARK_MAX::update_status_2(float position){
  status.position = position;
}

/*********************************************************************************************************
** Function name:           set_status_frame_period
** Descriptions:            Function to set period for SPARK MAX status frames
*********************************************************************************************************/
uint8_t SPARK_MAX::set_status_frame_period(const status_frame_id frame, const uint16_t period, MCP_CAN &CAN0){
  // Delay to ensure there is an available transmit buffer
  delay(50);
  uint8_t frame_data[STATUS_DLC];
  create_data(frame_data, &period, STATUS_WRITE_SIZE, STATUS_DLC);
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

/*********************************************************************************************************
** Function name:           data_to_float_32_bit
** Descriptions:            Converts four bytes (little-endian) to a IEEE floating point number
*********************************************************************************************************/
float SPARK_MAX::data_to_float_32_bit(uint8_t * data, uint8_t size){
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
void SPARK_MAX::parse_volt_and_amp(float * voltage, float * current, uint8_t *data){
  uint16_t voltage_p = (((uint16_t)data[6] & 0b00001111) << 8) | data[5]; // need to graph to find constant for converting into a float
  uint16_t current_p = ((uint16_t)data[7] << 4) | (data[6] >> 4); // need to graph to find constant for converting into a float
  *voltage = voltage_p;
  *current = current_p;
}

/*********************************************************************************************************
** Function name:           parse_status_frame_0
** Descriptions:            Function to parse SPARK MAX Periodic Status Frame 0
*********************************************************************************************************/
void SPARK_MAX::parse_status_frame_0(uint8_t * data){
  float applied_output = ((static_cast<int>(data[1]) << 8) | data[0]) / 32764.0; // Range = [-1, 1]

  // Check to see if send backwards (idk why this happens)
  if (abs(applied_output) > 1){
    applied_output = 2 - applied_output;
  }

  update_status_0(applied_output);
}

/*********************************************************************************************************
** Function name:           parse_status_frame_1
** Descriptions:            Function to parse SPARK MAX Periodic Status Frame 1
*********************************************************************************************************/
void SPARK_MAX::parse_status_frame_1(uint8_t * data, uint8_t size){
  float velocity = data_to_float_32_bit(data, size); // RPM
  uint8_t temperature = data[4]; // Celcius
  float voltage = 0.0;
  float current = 0.0;
  parse_volt_and_amp(&voltage, &current, data);

  update_status_1(velocity, temperature, voltage, current);
}

/*********************************************************************************************************
** Function name:           parse_status_frame_2
** Descriptions:            Function to parse SPARK MAX Periodic Status Frame 2
*********************************************************************************************************/
void SPARK_MAX::parse_status_frame_2(uint8_t * data, uint8_t size){
  float position = data_to_float_32_bit(data, size); // Rotations

  update_status_2(position);
}
