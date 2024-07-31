import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from rclpy.executors import SingleThreadedExecutor

class TwistRelay(Node):
    def __init__(self):
        super().__init__('twist_relay')
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.listener_callback, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel_relay', 10)

    def listener_callback(self, msg: Twist):
        self.publisher.publish(msg)
        self.get_logger().info('Relaying: Linear x: "%f", Angular z: "%f"' % 
                               (msg.linear.x, msg.angular.z))

class EnabledRelay(Node):
    def __init__(self):
        super().__init__('enabled_relay')
        self.subscription = self.create_subscription(
            Bool, 'enabled', self.listener_callback, 10)
        self.publisher = self.create_publisher(Bool, 'enabled_relay', 10)

    def listener_callback(self, msg: Bool):
        self.publisher.publish(msg)
        self.get_logger().info('Relaying: Enabled: "%s"' % str(msg.data))

class SensorRelay(Node):
    def __init__(self):
        super().__init__('sensor_relay')
        self.subscription = self.create_subscription(
            Bool, 'sensor_data', self.listener_callback, 10)
        self.publisher = self.create_publisher(Bool, 'sensor_data_relay', 10)

    def listener_callback(self, msg: Bool):
        self.publisher.publish(msg)
        self.get_logger().info('Relaying: Sensor data: "%s"' % str(msg.data))

def main(args=None):
    rclpy.init(args=args)
    
    twist_relay = TwistRelay()
    enabled_relay = EnabledRelay()
    sensor_relay = SensorRelay()
    
    # Create a single-threaded executor
    executor = SingleThreadedExecutor()

    # Add nodes to the executor
    executor.add_node(twist_relay)
    executor.add_node(enabled_relay)
    executor.add_node(sensor_relay)

    try:
        # Spin all nodes
        executor.spin()
    finally:
        # Destroy nodes
        twist_relay.destroy_node()
        enabled_relay.destroy_node()
        sensor_relay.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()