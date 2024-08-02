import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class TeleopPublisher(Node):
    def __init__(self):
        super().__init__('teleop_publisher')  # Initialize the node with the name 'teleop_publisher'

        from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange

        # Declare parameters with default values
        self.declare_parameter(
            'cmd_vel_pub_frequency', 
            50.0, 
            ParameterDescriptor(description='Frequency to publish cmd_vel messages in Hz'))

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
            ParameterDescriptor(description='Exponent for scaling joystick input'))  # Exponent for scaling joystick input

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

        # Initialize a Twist message
        self.twist_msg = Twist()
        
        # Prevent unused variable warning
        self.subscription

    def listener_callback(self, msg: Joy):
        # Map joystick axes to linear and angular velocities
        self.twist_msg.linear.x = msg.axes[1]  # Forward/backward movement
        self.twist_msg.angular.z = msg.axes[0]  # Left/right rotation

        # Log the updated Twist message data
        self.get_logger().info(f'Updated Twist: Linear x: {self.twist_msg.linear.x}, Angular z: {self.twist_msg.angular.z}')

    def timer_callback(self):
        x = self.twist_msg.linear.x
        y = self.twist_msg.angular.z
        deadzone = self.get_parameter('deadzone').get_parameter_value().double_value

        # Apply deadzone filtering
        if math.hypot(x, y) < deadzone:
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = 0.0
        else:
            # Apply scaling to joystick input
            power = self.get_parameter('joystick_power').get_parameter_value().integer_value
            self.twist_msg.linear.x = math.pow(x, power)
            self.twist_msg.angular.z = math.pow(y, power)
            
        self.publisher.publish(self.twist_msg)  # Publish the Twist message

        # Log the publishing event
        self.get_logger().info(f'Publishing: Linear x: {self.twist_msg.linear.x}, Angular z: {self.twist_msg.angular.z}')

def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of the TeleopPublisher node
    teleop_publisher = TeleopPublisher()

    # Spin the node to process callbacks
    rclpy.spin(teleop_publisher)

    # Cleanup and shutdown
    teleop_publisher.destroy_node()
    rclpy.shutdown()

# Entry point for the script
if __name__ == '__main__':
    main()
