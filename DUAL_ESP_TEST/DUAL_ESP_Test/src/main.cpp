#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>

#define CAN0_INT 4
MCP_CAN CAN0(12);

const uint32_t HEARTBEAT_ID = 0x2052C80;
    const uint8_t HEARTBEAT_DLC = 8;
    uint8_t heartbeat_data_enabled[8] = {255, 255, 255, 255, 255, 255, 255, 255};

void setup() {
  Serial.begin(115200);
  delay(2000);
  

  if(CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");
  CAN0.setMode(MCP_NORMAL);  // Set operation mode to normal so the MCP2515 sends acks to received data.
  CAN0.enOneShotTX();
}

void loop() {
  if(CAN0.sendMsgBuf(HEARTBEAT_ID, CAN_EXTID, HEARTBEAT_DLC, heartbeat_data_enabled) == CAN_OK){
    Serial.println("Sent Heartbeat");
  }
  else{
    Serial.println("!!!___FAILED HEARTBEAT___!!!");
  }
}
