/*
::::To Do::::
++++ DONE
---- TODO
~~~~ IDEA

---- Define explicit error codes for adding devices to arr and other functions
*/

#ifndef SPARK_MAX_H
#define SPARK_MAX_H

#include <Arduino.h>
#include <mcp_can.h>

#include "Comet_CAN_Common.h"
#include "CAN_Device_Interface.h"
#include "Comet_CAN_Helper.h"

/*********************************************************************************************************
** SPARK_MAX class
** For configuring and controlling SPARK MAXs
*********************************************************************************************************/
class SPARK_MAX : public ICAN_Device {
public:
    /*
    * Constructor. MUST BE CALLED AFTER THE CAN MODULE HAS BEEN SETUP
    */
    SPARK_MAX(uint8_t device_id) : device_id(device_id){
    }

    byte initialize_SPARK_MAX(Comet_CAN_Helper &CAN_Helper,  MCP_CAN &CAN0, const uint16_t period0 = 25, const uint16_t period1 = 100, 
                const uint16_t period2 = 100, const uint16_t period3 = 1000, const uint16_t period4 = 1000 ){

        this->period0 = period0;
        this->period1 = period1;
        this->period2 = period2;
        this->period3 = period3;
        this->period4 = period4;

        current_mode = control_mode::NONE;

        current_control_frame = empty_frame;

        active = true;

        if (CAN_Helper.add_to_CAN_dev_arr(this) == CAN_OK){
            set_all_status_frame_periods(CAN0, period0, period1, period2, period3, period4);
            return CAN_OK;
        }
        else{
            return CAN_FAIL;
        }

    }

    /*
    * Functions
    */
    bool is_FRC() const override {
        return true; // Example implementation
    }

    uint8_t get_device_id() const override {
        if (is_FRC()) {
            return device_id;
        } else {
            return 0; // Return a default value or handle appropriately if ID cannot be reported
                        // There should be no FRC devices with a device id equal to 0!
        }
    }

    can_frame get_current_frame() const override {
        return current_control_frame; 
    }

    void clear_current_frame() override {
        current_control_frame = empty_frame;
    }

    bool is_active() const override{
        return active;
    }

    void set_active(bool input) override{
        active = input;
    }

    void parse_CAN_frame(u_int32_t rxId, u_int8_t len, u_int8_t *rxBuf) override{
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

    uint8_t set_control_frame(const control_mode mode, const float setpoint); // Command SPARK MAX output

    void update_status_0(float applied_output); // Updates the applied ouput
    void update_status_1(float velocity, float temperature, float voltage, float current); // Updates the Velocity, Temperature, Voltage, and Current
    void update_status_2(float position); // Updates the Position

    SPARK_MAX_status get_status(){
        return status;
    }

    String to_string(){
        return "Device id: " + String(get_device_id()) + 
                "\nApplied output: " + String(status.applied_output) +
                "Velocity: " + String(status.velocity) +
                "Temperature: " + String(status.temperature) +
                "Voltage: " + String(status.voltage) +
                "Current: " + String(status.current) +
                "Position: " + String(status.position);
    }


    // Default destructor
    ~SPARK_MAX(){
    }

private:
    /*
    * Constants/variables
    */
    const uint8_t device_id;
    control_mode current_mode;
    can_frame current_control_frame = empty_frame; // Used to store the most recent control frame
    bool active = false;
    SPARK_MAX_status status = empty_status;

    u_int16_t period0, period1, period2, period3, period4;

    /*
    * Control Frame
    */
    static constexpr uint8_t CONTROL_DLC = 8;
    static constexpr uint8_t CONTROL_WRITE_SIZE = 4; // Size (bytes) of actual data written into the Data Window

    /*
    * Status Frame
    */
    static constexpr uint8_t STATUS_DLC = 2;
    static constexpr uint8_t STATUS_WRITE_SIZE = 2; // Size (bytes) of actual data written into the Data Window

    uint8_t set_status_frame_period(const status_frame_id frame, const uint16_t period, MCP_CAN &CAN0); // Set period for SPARK MAX status frames

    void set_all_status_frame_periods(MCP_CAN &CAN0, u_int16_t period0, u_int16_t period1, u_int16_t period2, u_int16_t period3, u_int16_t period4);

    float data_to_float_32_bit(uint8_t *data, uint8_t size); // Converts four bytes (little-endian) to a IEEE floating point number
    void parse_volt_and_amp(float * voltage, float * current, uint8_t *data); // Converts 12 bits (little-endian) to a floating point number
    void parse_status_frame_0(uint8_t *data);                 // Parse status frame 0
    void parse_status_frame_1(uint8_t *data, uint8_t size);   // Parse status frame 1
    void parse_status_frame_2(uint8_t *data, uint8_t size);   // Parse status frame 2
};

#endif
