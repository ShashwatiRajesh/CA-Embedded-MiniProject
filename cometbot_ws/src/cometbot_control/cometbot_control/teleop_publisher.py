"""
::::To Do::::
++++ DONE
---- TODO
**** TODO Important

---- Remove locking. The node's callback are multually exclusive by default 

"""

import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange

class TeleopPublisher(Node):
    def __init__(self):
        super().__init__('teleop_publisher')  # Initialize the node with the name 'teleop_publisher'

        # Declare parameters with default values
        self.declare_parameter(
            'cmd_vel_pub_frequency', 
            20.0, 
            ParameterDescriptor(
                description='Frequency to publish cmd_vel messages in Hz. \nMAKE SURE JOY_NODE IS ALSO RUNNING AT SAME OR HIGHER FREQUENCY',
                floating_point_range=[
                    FloatingPointRange(from_value=0.0, to_value=100.0)
                ]
            )
        )

        self.declare_parameter(
            'deadzone',
            0.1,
            ParameterDescriptor(
                description='Deadzone for joystick input',
                floating_point_range=[
                    FloatingPointRange(from_value=0.0, to_value=1.0, step=0.01)
                ]
            )
        )

        self.declare_parameter(
            'joystick_power', 
            3, 
            ParameterDescriptor(
                description='Exponent for scaling joystick input', 
                integer_range=[
                    IntegerRange(from_value = 1, to_value = 9, step = 2)
                ]
            )
        )

        # Create a subscription to the 'joy' topic
        self.subscription = self.create_subscription(
            Joy,  # Message type: Joy
            'joy',  # Topic name: 'joy'
            self.listener_callback,  # Callback function to handle incoming messages
            10  # QoS profile depth
        )

        # Create a publisher for the 'cmd_vel' topic
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Set up a timer to publish Twist messages at the specified frequency
        timer_frequency = self.get_parameter('cmd_vel_pub_frequency').get_parameter_value().double_value
        self.timer = self.create_timer(1.0 / timer_frequency, self.timer_callback)

        # Only edit in subscriber
        self.input = Twist()
        self.output = Twist()

        self.deadzone = self.get_parameter('deadzone').get_parameter_value().double_value
        self.power = self.get_parameter('joystick_power').get_parameter_value().integer_value
        
        # Prevent unused variable warning
        self.subscription
        self.get_logger().info('Initialized')

    def listener_callback(self, msg: Joy):
        # Map joystick axes to linear and angular velocities
        self.input.linear.x = msg.axes[1]  # Forward/backward movement
        self.input.angular.z = msg.axes[3]  # Left/right rotation

        # Log the updated Twist message data
        self.get_logger().info(f'Subscribing: Linear x: {self.input.linear.x}, Angular z: {self.input.angular.z}')

    def timer_callback(self):
        self.output.linear.x = self.input.linear.x
        self.output.angular.z = self.input.angular.z
        

        # Apply deadzone filtering
        if math.hypot(self.output.linear.x, self.output.angular.z) < self.deadzone:
            self.output.linear.x = 0.0
            self.output.angular.z = 0.0
        else:
            # Apply scaling to joystick input
            
            self.output.linear.x = math.pow(self.output.linear.x, self.power)
            self.output.angular.z = math.pow(self.output.angular.z, self.power)
            
        self.publisher.publish(self.output)  # Publish the Twist message

        # Log the publishing event
        self.get_logger().info(f'Publishing: Linear x: {self.output.linear.x}, Angular z: {self.output.angular.z}')

def main(args=None):
    try:
        # Initialize the ROS 2 Python client library
        rclpy.init(args=args)

        # Create an instance of the TeleopPublisher node
        teleop_publisher = TeleopPublisher()

        # Spin the node to process callbacks
        rclpy.spin(teleop_publisher)
    except KeyboardInterrupt:
        pass


# Entry point for the script
if __name__ == '__main__':
    main()
