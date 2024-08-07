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
        mutex = xSemaphoreCreateMutex();
    }

    void initialize_SPARK_MAX(Comet_CAN_Helper &CAN_Helper,  MCP_CAN &CAN0, const uint16_t period0 = 100, const uint16_t period1 = 100, 
                const uint16_t period2 = 100, const uint16_t period3 = 1000, const uint16_t period4 = 1000 ){

        this->period0 = period0;
        this->period1 = period1;
        this->period2 = period2;
        this->period3 = period3;
        this->period4 = period4;

        current_mode = control_mode::NONE;

        current_control_frame.dlc = CONTROL_DLC;
        current_control_frame.ext = 1;

        CAN_Helper.add_to_CAN_dev_arr(this);

        set_all_status_frame_periods(CAN0, period0, period1, period2, period3, period4);
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
        }
    }

    can_frame get_current_frame() const override {
        xSemaphoreTake(mutex, portMAX_DELAY);
        can_frame current_frame = current_control_frame;
        xSemaphoreGive(mutex);
        return current_frame; 
    }

    void clear_current_frame() override {
        xSemaphoreTake(mutex, portMAX_DELAY);
        current_control_frame = empty_frame;
        xSemaphoreGive(mutex);
    }

    uint8_t set_control_frame(const control_mode mode, const float setpoint); // Command SPARK MAX output

    // Default destructor
    ~SPARK_MAX(){
        // Delete the mutexes
        if (mutex != NULL) {
            vSemaphoreDelete(mutex);
        }
    }

private:
    /*
    * Constants/variables
    */
    const uint8_t device_id;
    control_mode current_mode;
    can_frame current_control_frame = empty_frame; // Used to store the most recent control frame
    SemaphoreHandle_t mutex;

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
};

#endif
