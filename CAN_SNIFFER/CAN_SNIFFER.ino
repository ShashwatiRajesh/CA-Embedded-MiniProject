#include <SPI.h>
#include <mcp2515.h>

MCP2515 mcp2515(10);
struct can_frame canMsg;


const bool EXTENDED_CAN_ID = true;

// Mask everything but device ID bits (last 6). Follows FRC CAN Protocol
const uint32_t DEVICE_ID_MASK = 0xFFFFFFC0; // 0xFFFFFFC0

// SPARK MAX periodic CAN frames
const uint32_t PERIODIC_STATUS_0_FILTER = 0x82051800;
const uint32_t PERIODIC_STATUS_1_FILTER = 0x82051840;
const uint32_t PERIODIC_STATUS_2_FILTER = 0x82051880;
const uint32_t EMPTY_FILTER = 0x00000000;

// Function Prototypes
void parse_status_frame_0(uint8_t * data);
void parse_status_frame_1(uint8_t * data, uint8_t size);
void parse_status_frame_2(uint8_t * data, uint8_t size);
float data_to_float(uint8_t * data, uint8_t size);


void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("CAN SNIFFER");

  // Setup for MCP 2515
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  
  mcp2515.setFilterMask(MCP2515::MASK0, EXTENDED_CAN_ID, DEVICE_ID_MASK);
  mcp2515.setFilterMask(MCP2515::MASK1, EXTENDED_CAN_ID, DEVICE_ID_MASK);

  // Works better to have less frequent messages on mask0 (RFX0 and RFX1)
  mcp2515.setFilter(MCP2515::RXF0, EXTENDED_CAN_ID, PERIODIC_STATUS_2_FILTER);
  mcp2515.setFilter(MCP2515::RXF1, EXTENDED_CAN_ID, PERIODIC_STATUS_1_FILTER);
  mcp2515.setFilter(MCP2515::RXF2, EXTENDED_CAN_ID, PERIODIC_STATUS_0_FILTER);
  mcp2515.setFilter(MCP2515::RXF3, EXTENDED_CAN_ID, EMPTY_FILTER);
  mcp2515.setFilter(MCP2515::RXF4, EXTENDED_CAN_ID, EMPTY_FILTER);
  mcp2515.setFilter(MCP2515::RXF5, EXTENDED_CAN_ID, EMPTY_FILTER);

  // Must go after all MCP2515 configuration
  mcp2515.setNormalMode();
}

void loop() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    Serial.print("Device ID : ");
    Serial.print(canMsg.can_id & 0x3F, DEC); // print Device ID
    Serial.print(" "); 
    
    /*
    // For non-binary prints
    for (int i = 0; i<canMsg.can_dlc; i++)  {  // print the data
      Serial.print(canMsg.data[i], BIN);
      Serial.print(" ");
    }*/

    // Always prints 8 bits
    for (int i = 0; i < canMsg.can_dlc; i++) {
        for (int j = 0; j < 8; j++) {
            Serial.print(bitRead(canMsg.data[i], 7 - j));
        }
        Serial.print(" ");
    }
    Serial.println();    


    // Handle data parsing for specific frames
    if ((canMsg.can_id & DEVICE_ID_MASK) == PERIODIC_STATUS_0_FILTER) {
      parse_status_frame_0(canMsg.data);
    }
    else if ((canMsg.can_id & DEVICE_ID_MASK) == PERIODIC_STATUS_1_FILTER) {
      parse_status_frame_1(canMsg.data, canMsg.can_dlc);
    }
    else if ((canMsg.can_id & DEVICE_ID_MASK) == PERIODIC_STATUS_2_FILTER) {
      parse_status_frame_2(canMsg.data, canMsg.can_dlc);
    }
    // Add more cases if necessary

  }
  else{
    //Serial.println("Error reading CAN message");
  }
}

void parse_status_frame_0(uint8_t * data){
  Serial.print("Applied output : ");
  Serial.println(((static_cast<int>(data[1]) << 8) | data[0]) / 32764.0); // Range = [-1, 1]
}

void parse_status_frame_1(uint8_t * data, uint8_t size){
  Serial.print("Velocity : ");
  Serial.println(data_to_float(data, size)); // RPM
}

void parse_status_frame_2(uint8_t * data, uint8_t size){
  Serial.print("Position : ");
  Serial.println(data_to_float(data, size)); // Rotations
}

// Converts four bytes (little-endian) to a IEEE floating point number
float data_to_float(uint8_t * data, uint8_t size){
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