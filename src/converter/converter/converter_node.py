import rclpy
from rclpy.node import Node
from ros_phoenix.msg import MotorControl
from interfaces.msg import FourMotorsControl

class MotorControlConverter(Node):
    def __init__(self):
        super().__init__('converter_node')
        # Subscribe using the FourMotorsControl msg type from the interfaces package
        self.subscription = self.create_subscription(
            FourMotorsControl,
            '/four_motors_control',
            self.listener_callback,
            10)

        self.publisher = {}
        for i in range(1, 5):
            topic_name = f'/talon{i}/set'
            self.publisher[i] = self.create_publisher(MotorControl, topic_name, 10)

    def listener_callback(self, msg):
        # Iterate through each motor's mode and value and publish them individually
        for i, (mode, value) in enumerate(zip(msg.modes, msg.values), start=1):
            new_msg = MotorControl()
            new_msg.mode = mode
            new_msg.value = value
            self.publisher[i].publish(new_msg)
            self.get_logger().info(f'Publishing to /talon{i}/set: mode={mode}, value={value}')

def main(args=None):
    rclpy.init(args=args)
    converter = MotorControlConverter()
    rclpy.spin(converter)
    converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
