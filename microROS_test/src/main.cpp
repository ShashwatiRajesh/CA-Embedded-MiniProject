/*
::::To Do::::
++++ DONE
---- TODO
**** TODO Important

++++ Convert Twist into motor output
++++ Change to set all the periodic status's at the constructor so it doesn't need to be stored
++++ thread safety
---- clean up code and comment
++++ Error handling (Restart program if major errored)

---- motor sensor publisher and CAN reader
---- look into sending more then 3 message duing CAN Core loop
---- Test without mutexs in Spark Max class. p sure the rclc executor is single threaded.
---- Change CAN device interface to include a get_is_enabled function so that if it is not being used the CAN core can skip it
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
#include <custom_messages/msg/spark_max_message.h>
#include <custom_messages/msg/robot_status_message.h>
#include "Comet_CAN_Helper.h"
#include "SPARK_MAX.h"
#include "Comet_CAN_Common.h"

/*
 * Function Prototypes
 */
void setup_timers();
void setup_publishers();
void setup_subscribers();
void setup_executor();
void initialize_vars();
void setup_CAN();
void update_robot_data_from_Spark_Max(SPARK_MAX *spark_max);

// Micro ROS

/*
 * Publishers
 */
rcl_publisher_t robot_data_publisher;
custom_messages__msg__RobotStatusMessage robot_data;
// Logger
rcl_publisher_t logging_publisher;
std_msgs__msg__String logger;
char log_message[256];  // Adjust size as needed\
// Debugging
char hearbeat_start_string[64];  // Adjust size as needed
unsigned long int start_time;
char cmd_vel_string[64];  // Adjust size as needed

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
rcl_timer_t robot_status_timer;
rcl_timer_t CAN_core_timer;
rcl_timer_t read_timer;

/*
 * CAN
 */
#define CAN0_INT 4
MCP_CAN CAN0(12);
Comet_CAN_Helper CAN_Helper(CAN0, CAN0_INT);
// To only send 1 disabled after an enable, because lack of heartbeat is also a disable.
bool was_enabled = false; 

/*
* SPARK MAXs
*/
SPARK_MAX drive_base_left = SPARK_MAX(11);
SPARK_MAX drive_base_right = SPARK_MAX(10);

/*
 * Other
 */
#define LED 2  // Onboard LED

/*
 * Macros to check the return value of RCL functions and CAN setup.
 */
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}} //   Enters error loop and restarts
#define CANCHECK(fn) {byte err = fn; if((err != CAN_OK)){error_loop();}} //   Enters error loop and restarts
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){} } // Allows program to keep running after error

/*
 * Blink for 5 seconds then reboot
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
 * !!!___ONLY USE FOR DEBUGGING___!!! TAKES ~10MS TO RUN
 */
void log_logging(const char *msg) {
  snprintf(log_message, sizeof(log_message), "[%lu ms] %s", millis(), msg);
  logger.data.size = strlen(log_message);
  logger.data.data = (char*)log_message;
  logger.data.capacity = logger.data.size + 1;
  RCSOFTCHECK(rcl_publish(&logging_publisher, &logger, NULL));
}

/*
* Updates a SPARK MAX message from a given SPARK MAX object
*/
void update_robot_data_from_Spark_Max(custom_messages__msg__SparkMaxMessage &spark_max_msg, SPARK_MAX *spark_max){
  SPARK_MAX_status status = spark_max->get_status();

  spark_max_msg.applied_output = status.applied_output;
  spark_max_msg.current = status.current;
  spark_max_msg.device_id = spark_max->get_device_id();
  spark_max_msg.position = status.position;
  spark_max_msg.temperature = status.temperature;
  spark_max_msg.velocity = status.velocity;
  spark_max_msg.voltage = status.voltage;
}

/*
 * Timer callback function to be called periodically
 */
void robot_status_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {  
  RCLC_UNUSED(last_call_time);  // Prevent unused variable warning
  if (timer != NULL) {
    robot_data.enabled = enabled.data;
    update_robot_data_from_Spark_Max(robot_data.left_drivebase, &drive_base_left);
    log_logging(drive_base_left.to_string().c_str());
    update_robot_data_from_Spark_Max(robot_data.right_drivebase, &drive_base_right);
    log_logging(drive_base_right.to_string().c_str());
    RCSOFTCHECK(rcl_publish(&robot_data_publisher, &robot_data, NULL));
  }
}

/*
 * Timer callback function to be called periodically
 */
void CAN_core_callback(rcl_timer_t * timer, int64_t last_call_time) {  
  RCLC_UNUSED(last_call_time);  // Prevent unused variable warning
  
  if (timer != NULL) {
    if (enabled.data){
      was_enabled = true;

      if(CAN_Helper.send_enabled_heartbeat() == CAN_OK){
        //log_logging("Heartbeat Sent Successfully");
      } else {
        //log_logging("Error Sending Heartbeat...!!!...");
      }

      //log_logging(CAN_Helper.send_message().c_str());
      CAN_Helper.send_message();

      //log_logging(CAN_Helper.send_message().c_str());
      CAN_Helper.send_message();

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
}

/*
 * Timer callback function for reading from the CAN buffer
 */
void read_callback(rcl_timer_t * timer, int64_t last_call_time){
  // If both recieve buffers are full the buffered messages are not overwritten. The user has to read the buffer before new information can be accepted
  RCLC_UNUSED(last_call_time);  // Prevent unused variable warning
  if (timer != NULL) {
    start_time = millis();
    log_logging(CAN_Helper.parse_CAN_frame().c_str());
    //log_logging(String("Took " + String(millis() - start_time) + " ms to run").c_str());
    //CAN_Helper.parse_CAN_frame();
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
    geometry_msgs__msg__Twist input;
    input.linear.x = msg->linear.x; // Up positive, down negative
    input.angular.z = msg->angular.z; // left positive, right negative

    // Normalize
    double sum = abs(input.linear.x) + abs(input.angular.z);
    if (sum > 1){
      input.linear.x = input.linear.x / sum;
      input.angular.z = input.angular.z / sum;
    }

    // Apply Output
    float output1 = input.linear.x + input.angular.z;
    float output2 = input.linear.x - input.angular.z;

    float left_output = 0.0;
    float right_output = 0.0;

    // if x(+), and z(-)

    if (input.linear.x > 0){
      left_output = output2;
      right_output = output1;
    }
    else if(input.linear.x < 0){
      left_output = output1;
      right_output = output2;
    }
    else{
      left_output = output2;
      right_output = output1;
    }

    right_output = -right_output;

    drive_base_left.set_control_frame(control_mode::Duty_Cycle_Set, left_output);
    drive_base_right.set_control_frame(control_mode::Duty_Cycle_Set, right_output);
    //snprintf(cmd_vel_string, sizeof(cmd_vel_string), "left_output: [%f ], right_output: [%f ]", left_output, right_output);
    //log_logging(cmd_vel_string);
  }
  
}

/*
 * Subscription callback function for enabled messages
 */
void enabled_callback(const void * msgin){
  // Cast received message to bool
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;

  if (msg != NULL){
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

  //Pins
  pinMode(CAN0_INT, INPUT);  // Configuring pin for /INT input
  pinMode(LED, OUTPUT);  // Set LED_PIN as output
  digitalWrite(LED, HIGH);  // Turn on the LED

  setup_CAN();
  
  // Setup Node. Shouldn't need to change.
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_node", microros_ns, &support));

  setup_timers();
  setup_publishers();
  setup_subscribers();
  setup_executor();
  
  initialize_vars();
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
  // Timer to control publisher for the "robot_data" topic
  RCCHECK(rclc_timer_init_default(
    &robot_status_timer,
    &support,
    RCL_MS_TO_NS(250),
    robot_status_timer_callback));

  // Timer for CAN_core
  RCCHECK(rclc_timer_init_default(
    &CAN_core_timer,
    &support,
    RCL_MS_TO_NS(25),               // Was 25ms
    CAN_core_callback));

  
  // Timer for reading from CAN buffer
  RCCHECK(rclc_timer_init_default(
    &read_timer,
    &support,
    RCL_MS_TO_NS(10),             // was 25ms
    read_callback));
}

/*
 * Setup publishers for various topics
 */
void setup_publishers(){
  // Create a publisher for the "robot_data" topic
  RCCHECK(rclc_publisher_init_default(
    &robot_data_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(custom_messages, msg, RobotStatusMessage),
    "robot_data"));

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

/*
 * Setup executor with # of handles
 */
void setup_executor(){
  // Create an executor, set number of handles, and add handles
  // Order added defines execution hierarchy (FIFO)
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &CAN_core_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &robot_status_timer))
  RCCHECK(rclc_executor_add_timer(&executor, &read_timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel, cmd_vel_callback, ON_NEW_DATA)); // or ALWAYS
  RCCHECK(rclc_executor_add_subscription(&executor, &enabled_subscriber, &enabled, enabled_callback, ALWAYS)); // or ALWAYS
}

/*
 * Initialize MCP2515 and setup CAN devices
 */
void setup_CAN(){
  // Initialize MCP2515 running at 16MHz with a baudrate of 1000kb/s and the masks and filters disabled.
  CANCHECK(CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ));
  
  CAN0.setMode(MCP_NORMAL);  // Set operation mode to normal so the MCP2515 sends acks to received data.

  // Explicitly disable One-Shot transmissions
  CAN0.disOneShotTX();
  

  delay(250);

  // To wake up the CAN bus
  CAN_Helper.send_disabled_heartbeat();

  // CAN DEVICES
  CANCHECK(drive_base_left.initialize_SPARK_MAX(CAN_Helper, CAN0));
  CANCHECK(drive_base_right.initialize_SPARK_MAX(CAN_Helper, CAN0));
  
}

/*
 * initialize global vars
 */
void initialize_vars(){
  robot_data.enabled = false;
  robot_data.left_drivebase.applied_output = 0.0;
  robot_data.left_drivebase.current = 0.0;
  robot_data.left_drivebase.device_id = 0;
  robot_data.left_drivebase.position = 0.0;
  robot_data.left_drivebase.velocity = 0.0;
  robot_data.left_drivebase.voltage = 0.0;
  
  robot_data.right_drivebase.applied_output = 0.0;
  robot_data.right_drivebase.current = 0.0;
  robot_data.right_drivebase.device_id = 0;
  robot_data.right_drivebase.position = 0.0;
  robot_data.right_drivebase.velocity = 0.0;
  robot_data.right_drivebase.voltage = 0.0;


  logger.data.size = 100;
  enabled.data = true; // Change to false by default once web GUI has been built (ONLY FOR TESTING)
  // may need to use something like std_msgs__msg__String__fini(&sub_msg); for messages that are arrays
}