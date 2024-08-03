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
rcl_publisher_t sensor_data_publisher;
rcl_publisher_t heartbeat_publisher;
rcl_publisher_t logging_publisher;  // Publisher for logging topic
std_msgs__msg__String logger;
std_msgs__msg__Bool sensor_data;
rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist cmd_vel;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t sensor_timer;  // Timer for sensor data
rcl_timer_t heartbeat_timer;  // Timer for heartbeat data
const char * microros_ns = "";

// CAN
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];

#define CAN0_INT 4 // Not 2 like it was previously, that pin is used to signal errors
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

// Function to log messages to the logging topic
void log_logging(const char *msg) {
  logger.data.data = (char*)msg;
  logger.data.size = strlen(msg);
  logger.data.capacity = logger.data.size + 1;
  rcl_publish(&logging_publisher, &logger, NULL);
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

// Subscription callback function to be called on cmd_vel_relay received
void subscription_callback(const void * msgin)
{
  // Cast received message to Twist
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  if (msg != NULL){
    // Process Twist
    cmd_vel.linear.x = msg->linear.x;
    cmd_vel.angular.z = msg->angular.z;
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

  // Create a publisher for the "heartbeat_data" topic
  RCCHECK(rclc_publisher_init_default(
    &heartbeat_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "heartbeat_data"));

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

  // Testing multiple timers
  RCCHECK(rclc_timer_init_default(
    &heartbeat_timer,
    &support,
    RCL_MS_TO_NS(25),
    heartbeat_timer_callback));

  // Create a subscriber for the "cmd_vel_relay" topic
  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel_relay"));

  // Create an executor, set number of handles, and add handles
  // Order added defines execution hierarchy (FIFO)
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &sensor_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &heartbeat_timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel, subscription_callback, ON_NEW_DATA)); // or ALWAYS

  sensor_data.data = false;
  cmd_vel.linear.x = 0;
  cmd_vel.angular.z = 0;
  logger.data.size = 100;
  // may need to use something like std_msgs__msg__String__fini(&sub_msg); for messages that are arrays
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin(&executor));
}
