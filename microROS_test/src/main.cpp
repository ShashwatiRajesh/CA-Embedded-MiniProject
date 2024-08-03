#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/twist.h>
#include <SPI.h>
#include <mcp_can.h>

// Micro ROS
//Publishers
rcl_publisher_t sensor_data_publisher;
std_msgs__msg__Bool sensor_data;
rcl_publisher_t logging_publisher;  // Publisher for logging topic
std_msgs__msg__String logger;


// Subscribers
rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist cmd_vel;
rcl_subscription_t enabled_subscriber;
std_msgs__msg__Bool enabled;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
const char * microros_ns = "";

// Timers
rcl_timer_t sensor_timer;
rcl_timer_t heartbeat_timer;
rcl_timer_t read_timer;


// CAN
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];

#define CAN0_INT 4
MCP_CAN CAN0(12);

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

// Mask everything but device ID bits (last 6). Follows FRC CAN Protocol
const uint32_t DEVICE_ID_MASK = 0xFFFFFFC0; // 0xFFFFFFC0

// SPARK MAX periodic CAN frames
const uint32_t PERIODIC_STATUS_0_FILTER = 0x82051800;
const uint32_t PERIODIC_STATUS_1_FILTER = 0x82051840;
const uint32_t PERIODIC_STATUS_2_FILTER = 0x82051880;
const uint32_t EMPTY_FILTER = 0x00000000;

// Heartbeat Frame
const uint32_t HEARTBEAT_ID = 0x2052C80;  // 0x2052C80
byte heartbeat_data[8] = {255, 255, 255, 255, 255, 255, 255, 255};
const uint8_t HEARTBEAT_DLC = 8;

// Other
#define LED 2  // Onboard LED

// Macro to check the return value of RCL functions and enter error loop if needed
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}} // Allows program to keep running after error

// Error loop to blink onboard LED if an error occurs
void error_loop(){
  int delay_times = 50;
  while(delay_times){
    digitalWrite(LED, !digitalRead(LED));
    delay(100);
    delay_times--;
  }
  ESP.restart();
}


// Functions to parse SPARK MAX Periodic Status Frames
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


// Function to log messages to the logging topic
void log_logging(const char *msg) {
  logger.data.data = (char*)msg;
  logger.data.size = strlen(msg);
  logger.data.capacity = logger.data.size + 1;
  RCSOFTCHECK(rcl_publish(&logging_publisher, &logger, NULL));
}

// Timer callback function to be called periodically
void sensor_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);  // Prevent unused variable warning
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&sensor_data_publisher, &sensor_data, NULL));
    sensor_data.data = !sensor_data.data;
  }
}

// Timer callback function to be called periodically
void heartbeat_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);  // Prevent unused variable warning
  if (timer != NULL) {
    if( CAN0.sendMsgBuf(HEARTBEAT_ID, 1, HEARTBEAT_DLC, heartbeat_data) == CAN_OK){
      log_logging("Message Sent Successfully!");
    } else {
      log_logging("Error Sending Message...");
    }
  }
}

// Timer callback function for reading from the CAN buffer
void read_callback(rcl_timer_t * timer, int64_t last_call_time){
  RCLC_UNUSED(last_call_time);  // Prevent unused variable warning
  if (timer != NULL) {
    if(!digitalRead(CAN0_INT))                         // If CAN0_INT pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
    
    // Handle data parsing for specific frames
    if ((rxId & DEVICE_ID_MASK) == PERIODIC_STATUS_0_FILTER) {
      parse_status_frame_0(rxBuf);
    }
    else if ((rxId & DEVICE_ID_MASK) == PERIODIC_STATUS_1_FILTER) {
      parse_status_frame_1(rxBuf, len);
    }
    else if ((rxId & DEVICE_ID_MASK) == PERIODIC_STATUS_2_FILTER) {
      parse_status_frame_2(rxBuf, len);
    }
    // Add more cases if necessary
  }
  }
}

// Subscription callback function to be called on cmd_vel_relay received
void cmd_vel_callback(const void * msgin)
{
  // Cast received message to Twist
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  if (msg != NULL){
    // Process Twist
    cmd_vel.linear.x = msg->linear.x;
    cmd_vel.angular.z = msg->angular.z;
  }
}

void enabled_callback(const void * msgin){
  // Cast received message to bool
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;

  if (msg != NULL){
    // Process Twist
    enabled.data = msg->data;
  }
}

void setup() {
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  // Initialize MCP2515 running at 8MHz with a baudrate of 1000kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK)
    log_logging("MCP2515 Initialized Successfully!");
  else
    log_logging("Error Initializing MCP2515...");

  CAN0.setMode(MCP_NORMAL);  // Set operation mode to normal so the MCP2515 sends acks to received data.

  pinMode(CAN0_INT, INPUT);  // Configuring pin for /INT input

  pinMode(LED, OUTPUT);  // Set LED_PIN as output
  digitalWrite(LED, HIGH);  // Turn on the LED

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(&node, "micro_ros_node", microros_ns, &support));

  // Create a publisher for the "sensor_data" topic
  RCCHECK(rclc_publisher_init_default(
    &sensor_data_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "sensor_data"));

  // Create a publisher for the "logging" topic
  RCCHECK(rclc_publisher_init_default(
    &logging_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "logging"));

  // Timer to control publisher for the "sensor_data" topic
  RCCHECK(rclc_timer_init_default(
    &sensor_timer,
    &support,
    RCL_MS_TO_NS(250),
    sensor_timer_callback));

  // Timer for heartbeat
  RCCHECK(rclc_timer_init_default(
    &heartbeat_timer,
    &support,
    RCL_MS_TO_NS(25),
    heartbeat_timer_callback));

  // Timer for reading from CAN buffer
  RCCHECK(rclc_timer_init_default(
    &read_timer,
    &support,
    RCL_MS_TO_NS(25),
    read_callback));

  // Create a subscriber for the "cmd_vel_relay" topic
  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel_relay"));

  // Create a subscriber for the "enabled" topic
  RCCHECK(rclc_subscription_init_default(
    &enabled_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "enabled"));

  // Create an executor, set number of handles, and add handles
  // Order added defines execution hierarchy (FIFO)
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &sensor_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &heartbeat_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &read_timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel, cmd_vel_callback, ON_NEW_DATA)); // or ALWAYS
  RCCHECK(rclc_executor_add_subscription(&executor, &enabled_subscriber, &enabled, enabled_callback, ON_NEW_DATA)); // or ALWAYS

  sensor_data.data = false;
  cmd_vel.linear.x = 0;
  cmd_vel.angular.z = 0;
  logger.data.size = 100;
  enabled.data = true; // Change to false by defauly once web GUI has been built (ONLY FOR TESTING)
  // may need to use something like std_msgs__msg__String__fini(&sub_msg); for messages that are arrays
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin(&executor));
}
