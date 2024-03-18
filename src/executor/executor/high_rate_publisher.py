import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from interfaces.msg import MotorControl, MotorStatus
import math, time


class HighRatePublisher(Node):
    def __init__(self):
        super().__init__('high_rate_publisher')
        qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(MotorControl, '/talon1/set', qos_profile)
        self.timer = self.create_timer(1.0 / 50.0, self.publish_cmd)  # run at 50Hz

        # Declare a parameter for the amplitude of the sine wave
        self.declare_parameter('amplitude', 0.2)

    def publish_cmd(self):
        # Retrieve the current amplitude parameter value
        amplitude = self.get_parameter('amplitude').value

        # Simulate a sine wave as input
        t = time.time()
        value = amplitude * math.sin(t)

        msg = MotorControl()
        msg.mode = 0
        msg.value = value
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: mode={msg.mode}, value={msg.value}')

def main(args=None):
    rclpy.init(args=args)
    node = HighRatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
