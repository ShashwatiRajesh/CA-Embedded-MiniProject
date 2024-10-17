/*
::::To Do::::
++++ DONE
---- TODO
~~~~ IDEA

---- Define explicit error codes for adding devices to arr and other functions
*/

#ifndef PDP_H
#define PDP_H

#include <Arduino.h>
#include <mcp_can.h>

#include "Comet_CAN_Common.h"
#include "CAN_Device_Interface.h"
#include "Comet_CAN_Helper.h"

/*********************************************************************************************************
** PDP class
** For configuring and controlling SPARK MAXs
*********************************************************************************************************/
class PDP : public ICAN_Device {
public:
    /*
    * Constructor. MUST BE CALLED AFTER THE CAN MODULE HAS BEEN SETUP
    */
   
    PDP(){
    }

    void initialize_PDP(Comet_CAN_Helper &CAN_Helper,  MCP_CAN &CAN0){

        active = true;

    }
    /*
    * Functions
    */
    bool is_FRC() const override {
        return true;
    }

    uint8_t get_device_id(){
        return 0; // PDP has no device ID
    }

    bool is_active() const override{
        return active;
    }

    void set_active(bool input) override{
        active = input;
    }

    void parse_CAN_frame(u_int32_t rxId, u_int8_t len, u_int8_t *rxBuf) override{
        // Handle data parsing for specific frames
        if ((rxId & FRC_dev_id_mask) == STATUS_1) {
          parse_status_frame_1(rxBuf, len);
        } else if ((rxId & FRC_dev_id_mask) == STATUS_2) {
          parse_status_frame_2(rxBuf, len);
        } else if ((rxId & FRC_dev_id_mask) == STATUS_3) {
          parse_status_frame_3(rxBuf, len);
        }
        
        
        // Add more cases if necessary
    }

    PDP_status get_status(){
        return status;
    }

    String to_string(){
        double totalCurrent = 0.0;
        String currents = "\nCurrents: {";
        for (int i = 0; i < 15; i++){
            currents+=String(status.currents[i]) + ", ";
            totalCurrent+=status.currents[i];
        }
        return "Device id: " + String(get_device_id()) + 
                "\nTemperature: " + String(status.temperature) +
                currents + String(status.currents[15]) + "}" +
                "\nTotal Current: " + String(totalCurrent + status.currents[15]);
    }


    // Default destructor
    ~PDP(){
    }

private:
    /*
    * Constants/variables
    */
    PDP_status status = empty_pdp_status;
    bool active = false;

    const float kCurrentScalar = 0.125f;

    void parse_status_frame_1(uint8_t *data, uint8_t size);   // Parse status frame 1
    void parse_status_frame_2(uint8_t *data, uint8_t size);   // Parse status frame 2
    void parse_status_frame_3(uint8_t *data, uint8_t size);   // Parse status frame 3
};

#endif
