import rclpy  # Import the ROS 2 Python library
from rclpy.node import Node  # Import the Node class from rclpy
from sensor_msgs.msg import Joy  # Import the Joy message from the sensor_msgs package

# Define a class that inherits from Node, which will subscribe to the 'joy' topic
class JoySubscriber(Node):
    def __init__(self):
        # Initialize the Node with the name 'joy_subscriber'
        super().__init__('joy_subscriber')
        
        # Create a subscription to the 'joy' topic with a QoS profile of 10
        self.subscription = self.create_subscription(
            Joy,  # Message type: Joy
            'joy',  # Topic name: joy
            self.listener_callback,  # Callback function to handle messages
            10)  # QoS profile depth
        
        # Prevent unused variable warning
        self.subscription

    # Define the callback function to process incoming Joy messages
    def listener_callback(self, msg):
        # Log the axes and buttons data from the Joy message
        self.get_logger().info('Axes: "%s", Buttons: "%s"' % (msg.axes, msg.buttons))

# Main function to initialize and spin the node
def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of the JoySubscriber node
    joy_subscriber = JoySubscriber()
    
    # Spin the node to keep it active and processing incoming messages
    rclpy.spin(joy_subscriber)
    
    # Destroy the node explicitly
    joy_subscriber.destroy_node()
    
    # Shutdown the ROS 2 Python client library
    rclpy.shutdown()

# Entry point for the script
if __name__ == '__main__':
    # Call the main function
    main()

