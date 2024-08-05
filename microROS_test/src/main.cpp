/*
::::To Do::::
++++ DONE
---- TODO

---- Convert Twist into motor output
---- Built in ROS publisher for motor data????
      should be handled individuallly
---- redo single message testing
  both how often and if it needs different data
*/

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
#include <Comet_CAN_Helper.h>

/*
 * Function Prototypes
 */
void setup_timers();
void setup_publishers();
void setup_subscribers();

// Micro ROS

/*
 * Publishers
 */
rcl_publisher_t sensor_data_publisher;
std_msgs__msg__Bool sensor_data;
rcl_publisher_t logging_publisher;
std_msgs__msg__String logger;

/*
 * Subscribers
 */
rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist cmd_vel;
rcl_subscription_t enabled_subscriber;
std_msgs__msg__Bool enabled;

/*
 * ROS Core
 */
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
const char * microros_ns = "";

/*
 * Timers
 */
rcl_timer_t sensor_timer;
rcl_timer_t heartbeat_timer;
rcl_timer_t read_timer;

/*
 * CAN
 */
#define CAN0_INT 4
MCP_CAN CAN0(12);
Comet_CAN_Helper CAN_Helper(CAN0);
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
bool was_enabled = false;
uint8_t control_data_testing[8] = {0,0,0,0,0,0,0,0};

/*
* SPARK MAXs
*/
SPARK_MAX drive_base_left = SPARK_MAX(11, &CAN_Helper, CAN0);
SPARK_MAX drive_base_right = SPARK_MAX(10, &CAN_Helper, CAN0);

/*
 * Other
 */
#define LED 2  // Onboard LED

/*
 * Macro to check the return value of RCL functions and enter error loop if needed
 */
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){} } // Allows program to keep running after error

/*
 * Error loop to blink onboard LED if an error occurs
 */
void error_loop(){
  int delay_times = 50;
  while(delay_times){
    digitalWrite(LED, !digitalRead(LED));
    delay(100);
    delay_times--;
  }
  ESP.restart();
}

/*
 * Function to log messages to the logging topic
 */
void log_logging(const char *msg) {
  logger.data.data = (char*)msg;
  logger.data.size = strlen(msg);
  logger.data.capacity = logger.data.size + 1;
  RCSOFTCHECK(rcl_publish(&logging_publisher, &logger, NULL));
}

/*
 * Timer callback function to be called periodically
 */
void sensor_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {  
  RCLC_UNUSED(last_call_time);  // Prevent unused variable warning
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&sensor_data_publisher, &sensor_data, NULL));
    sensor_data.data = !sensor_data.data;
  }
}

/*
 * Timer callback function to be called periodically
 */
void heartbeat_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {  
  RCLC_UNUSED(last_call_time);  // Prevent unused variable warning

  // Testing to see if a control message by itself will fail
  if(CAN0.sendMsgBuf(0x205008a, CAN_EXTID, 8, control_data_testing) == CAN_OK){
        log_logging("send_control_frame Sent Successfully");
      } else {
        log_logging("Error Sending send_control_frame...!!!...");
      }

  /*
  // Commented out logging to prevent delays
  if (timer != NULL) {
    if (enabled.data){
      was_enabled = true;
      if(CAN_Helper.send_enabled_heartbeat() == CAN_OK){
        //log_logging("Heartbeat Sent Successfully");
      } else {
        log_logging("Error Sending Heartbeat...!!!...");
      }
    }
    else{
      if (was_enabled){
        was_enabled = false;
        if(CAN_Helper.send_disabled_heartbeat() == CAN_OK){
          //log_logging("Message Sent Successfully!");
        } else {
          //log_logging("Error Sending Message...");
        }
      }
    }
  }
  */
}

/*
 * Timer callback function for reading from the CAN buffer
 */
void read_callback(rcl_timer_t * timer, int64_t last_call_time){
  RCLC_UNUSED(last_call_time);  // Prevent unused variable warning
  if (timer != NULL) {
    if(!digitalRead(CAN0_INT)) { // If CAN0_INT pin is low, read receive buffer
      CAN0.readMsgBuf(&rxId, &len, rxBuf);
      
      // Handle data parsing for specific frames
      if ((rxId & CAN_Helper.DEVICE_ID_MASK) == status_0) {
        CAN_Helper.parse_status_frame_0(rxBuf);
      } else if ((rxId & CAN_Helper.DEVICE_ID_MASK) == status_1) {
        CAN_Helper.parse_status_frame_1(rxBuf, len);
      } else if ((rxId & CAN_Helper.DEVICE_ID_MASK) == status_2) {
        CAN_Helper.parse_status_frame_2(rxBuf, len);
      }
      // Add more cases if necessary
    }
  }
}
 
/*
 * Subscription callback function to be called on cmd_vel_relay received
 */
void cmd_vel_callback(const void * msgin) {
  // Cast received message to Twist
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  if (msg != NULL) {
    // Process Twist
    cmd_vel.linear.x = msg->linear.x;
    cmd_vel.angular.z = msg->angular.z;

    /*
    uint8_t control_data_testing[8] = {0,0,0,0,0,0,0,0};
    // Test motor commands
    if(CAN0.sendMsgBuf(0x205004a, CAN_EXTID, 8, control_data_testing) == CAN_OK){
        //log_logging("send_control_frame Sent Successfully");
      } else {
        log_logging("Error Sending send_control_frame...!!!...");
      }
    // Send heartbeat inbetween commands to prevent from dropping it
    // CAN0.sendMsgBuf(HEARTBEAT_ID, 1, HEARTBEAT_DLC, heartbeat_data_enabled);
    if(CAN0.sendMsgBuf(0x205004b, CAN_EXTID, 8, control_data_testing) == CAN_OK){
        //log_logging("send_control_frame Sent Successfully");
      } else {
        log_logging("Error Sending send_control_frame...!!!...");
      }
    */

    /*
    ::::Isues::::
    Only one controller (MCP2515) at a time seems to work. Not sure why it's not possible
      Can't split duties and have one do heartbeat while the other does control

    Sending commands besides the heartbeat seems to cause it to not send fast enough and
      momentarily disable the motors

    Only control_frames are failing to send???
    
    REV Hardware client still magically fixes all issues

    ONLY THE CONTROL FRAMES AND DISABLE FRAMES ARE ERRORING. THE HEARTBEAT DOES NOT FAIL NEARLY AS MUCH
    MAYBE ONCE EVERY MINUTE OR TWO.
    THE OTHERS FAIL EVERY COUPLE OF SECONDS
    sending one message testing
      

    ::::Ideas::::

    PROBABLY GOOD PRACTICE
    Manually make a queue to send to the MCP and add a heartbeat between every command

    Try to send as few other CAN messages (besides heartbeat) as possible

    clear msg buffer before sending heartbeat frame
      ---- Aborttx() works (deletes stuff) but doesn't seems to be deleting everythin, even the heartbeat
        Means the heartbeat command is queued before the previous one is sent?

    try oneshot mode
      ---- Helped, flashing was less frequent, but still present (once every few seconds)
        Don't want oneshot mode enabled to heartbeat frames since it's important that those get transmitted
      try oneshot mode only with control commands
      ---- Worse than just leaving oneshot mode enabled

    Control just one spark max and take the other off the CAN loop
      ---- No change

    Increase timer callback frequency for the heartbeat timer
      ---- Didn't seem to help. Looked like it made it worse.
        Could be clogging up the buffer?
        heartbeat timer callback is interrupting or running at the same time and causing issues?

    16MHz crystal oscillator
      ---- Tuesday

    Different CAN module (Would need different library)

    Try ESP internal reciever and external transceiver again (idk how to get to work)

    Edit MCP Library and reserve a transmit buffer for heartbeat and set the priority to level 4 (highest)
    */
  }
}

/*
 * Subscription callback function for enabled messages
 */
void enabled_callback(const void * msgin){
  // Cast received message to bool
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;

  if (msg != NULL){
    // Process Enabled
    enabled.data = msg->data;
  }
}

/*
 * Setup function to initialize components
 */
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

  setup_timers();
  setup_publishers();
  setup_subscribers();

  // Create an executor, set number of handles, and add handles
  // Order added defines execution hierarchy (FIFO)
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &heartbeat_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &sensor_timer))
  RCCHECK(rclc_executor_add_timer(&executor, &read_timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel, cmd_vel_callback, ON_NEW_DATA)); // or ALWAYS
  RCCHECK(rclc_executor_add_subscription(&executor, &enabled_subscriber, &enabled, enabled_callback, ALWAYS)); // or ALWAYS

  sensor_data.data = false;
  cmd_vel.linear.x = 0;
  cmd_vel.angular.z = 0;
  logger.data.size = 100;
  enabled.data = true; // Change to false by default once web GUI has been built (ONLY FOR TESTING)
  // may need to use something like std_msgs__msg__String__fini(&sub_msg); for messages that are arrays
}

/*
 * Loop function that runs repeatedly
 */
void loop() {
  RCSOFTCHECK(rclc_executor_spin(&executor));
}

/*
 * Setup timers for various tasks
 */
void setup_timers(){
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
    RCL_MS_TO_NS(10),               // Was 25ms
    heartbeat_timer_callback));

  // Timer for reading from CAN buffer
  RCCHECK(rclc_timer_init_default(
    &read_timer,
    &support,
    RCL_MS_TO_NS(25),
    read_callback));
}

/*
 * Setup publishers for various topics
 */
void setup_publishers(){
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
    "micro_ROS_logging"));
}

/*
 * Setup subscribers for various topics
 */
void setup_subscribers(){
  // Create a subscriber for the "cmd_vel_relay" topic
  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // Create a subscriber for the "enabled" topic
  RCCHECK(rclc_subscription_init_default(
    &enabled_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "enabled"));
}
