import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class TeleopPublisher(Node):
    def __init__(self):
        super().__init__('teleop_publisher')  # Renamed node for clarity

        # Create a subscription to the 'joy' topic with a QoS profile of depth 10
        self.subscription = self.create_subscription(
            Joy,  # Message type: Joy
            'joy',  # Topic name: 'joy'
            self.listener_callback,  # Callback function to handle incoming messages
            10)  # QoS profile depth

        # Create a publisher for the 'cmd_vel' topic
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Set up a timer to publish Twist messages at set frequency
        timer_frequency = 50.0 #hz
        self.timer = self.create_timer(1.0 / timer_frequency, self.timer_callback)

        # Create an initial Twist message
        self.twist_msg = Twist()
        
        # Prevent unused variable warning
        self.subscription

    def listener_callback(self, msg: Joy):
        # Map joystick axes to linear and angular velocities
        # These axis indices depend on the joystick configuration
        self.twist_msg.linear.x = msg.axes[1]  # Forward/backward movement
        self.twist_msg.angular.z = msg.axes[0]  # Left/right rotation

        # Log the generated Twist message data
        self.get_logger().info(
            'Updated Twist: Linear x: "%f", Angular z: "%f"' % 
            (self.twist_msg.linear.x, self.twist_msg.angular.z)
        )

    def timer_callback(self):
        # Publish the Twist message at the timer's frequency
        self.publisher.publish(self.twist_msg)
        # Log the publishing event
        self.get_logger().info(
            'Publishing: Linear x: "%f", Angular z: "%f"' % 
            (self.twist_msg.linear.x, self.twist_msg.angular.z)
        )

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
