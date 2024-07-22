#include <SPI.h>
#include <mcp2515.h>

MCP2515 mcp2515(10);

enum control_mode {
  Duty_Cycle_Set = 0x2050080,
  Speed_Set = 0x2050480,
  Smart_Velocity_Set = 0x20504C0,
  Position_Set = 0x2050C80,
  Voltage_Set = 0x2051080,
  Current_Set = 0x20510C0,
  Smart_Motion_Set = 0x2051480
};

enum status_frame_id {
  status_0 = 0x2051800,
  status_1 = 0x2051840,
  status_2 = 0x2051880,
  status_3 = 0x20518C0,
  status_4 = 0x2051900
};

// Heartbeat Frame
const uint32_t HEARTBEAT_ID = 0x2052C80;  // 0x2052C80
const uint8_t HEARTBEAT_DATA[8] = { 255, 255, 255, 255, 255, 255, 255, 255 };
struct can_frame heartbeat_frame;
const uint8_t HEARTBEAT_DLC = 8;

// Control Frame
struct can_frame control_frame;
const uint8_t CONTROL_DLC = 8;
const uint8_t CONTROL_WRITE_SIZE = 4; // Size (bytes) of actual data written into the Data Window

// Status Frame
struct can_frame status_frame;
const uint8_t STATUS_DLC = 2;
const uint8_t STATUS_WRITE_SIZE = 2; // Size (bytes) of actual data written into the Data Window

// Function Prototypes
void pack_data(can_frame &frame, const uint8_t *data, const int size);
void create_data(const void *data, byte *frame_data, const uint8_t write_size, const uint8_t dlc);
void send_control_frame(const uint32_t device_id, const control_mode mode, const float setpoint);


void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  Serial.println("CAN CONTROLLER");


  /* 
  Setup predefined CAN Messages following the SPARK MAX CAN Protocol docmentation

  NOTE: All CAN frames are have an extended CAN ID so all .can_id assignments need to have the 
  | CAN_EFF_FLAG set. 
  */ 

  // Heartbeat
  heartbeat_frame.can_id = HEARTBEAT_ID | CAN_EFF_FLAG;
  heartbeat_frame.can_dlc = HEARTBEAT_DLC;
  pack_data(heartbeat_frame, HEARTBEAT_DATA, HEARTBEAT_DLC);

  // Control
  control_frame.can_dlc = CONTROL_DLC;

  // Status
  status_frame.can_dlc = STATUS_DLC;

  // Setup for MCP 2515
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode(); // Must go after all MCP2515 configuration

  // Synchronize all REV motor controllers
  // Not doing so may be causing txrx errors
  // send_CAN_packet(0x20524c0);

  /*
  Process for updateing period of periodic status frames
  set_status_frame_period(11, status_1, 100);
  */
}

void loop() {

  Serial.print("Sending heartbeat frame with error : ");
  Serial.println(mcp2515.sendMessage(MCP2515::TXB1, &heartbeat_frame));

  send_control_frame(10, Duty_Cycle_Set, 0);
}

/*
Create_data and pack_data should be combined into one function and the intermediary 
array should be cutout.
*/
void pack_data(can_frame &frame, const uint8_t *data, const int size) {
  // Writes data into the CAN frame
  for (int i = 0; i < size; i++) {
    frame.data[i] = data[i];
  }
}

void create_data(const void *data, byte *frame_data, const uint8_t write_size, const uint8_t dlc) {
  // Copy data to frame_data (little-Endian)
  const byte *data_arr = static_cast<const byte *>(data);
  for (int i = 0; i < write_size; i++) {
    frame_data[i] = data_arr[i];
  }

  // Fill remaining space with zeros
  for (int i = write_size; i < dlc; i++) {
    frame_data[i] = 0;
  }
}

// Future versions should allow user to select the PID profile
void send_control_frame(const uint32_t device_id, const control_mode mode, const float setpoint) {
  Serial.print("Sending control frame with error : ");
  control_frame.can_id = mode + device_id | CAN_EFF_FLAG;  // Don't forget | CAN_EFF_FLAG
  
  // To be combined
  byte control_data[CONTROL_DLC];
  create_data(&setpoint, control_data, CONTROL_WRITE_SIZE, CONTROL_DLC);
  pack_data(control_frame, control_data, CONTROL_DLC);

  Serial.println(mcp2515.sendMessage(MCP2515::TXB1, &control_frame));
}

void set_status_frame_period(const uint32_t device_id, const status_frame_id frame, const uint16_t period) {
  Serial.print("Setting frame period with error : ");
  status_frame.can_id = frame + device_id | CAN_EFF_FLAG;  // Don't forget | CAN_EFF_FLAG
  
  // To be combined
  byte status_data[STATUS_DLC];
  create_data(&period, status_data, STATUS_WRITE_SIZE, STATUS_DLC);
  pack_data(status_frame, status_data, STATUS_DLC);

  Serial.println(mcp2515.sendMessage(MCP2515::TXB1, &status_frame));
}