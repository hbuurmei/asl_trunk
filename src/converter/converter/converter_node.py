import rclpy
from rclpy.node import Node
from interfaces.msg import TrunkMarkers
from mocap4r2_msgs.msg import Markers

class ConverterNode(Node):
    def __init__(self):
        super().__init__('converter_node')
        self.publisher = self.create_publisher(TrunkMarkers, '/trunk_markers', 10)
        self.subscription = self.create_subscription(
            Markers,
            '/markers',
            self.marker_callback,
            10)
        self.get_logger().info('Mocap converter node has started.')

    def marker_callback(self, msg):
        trunk_msg = TrunkMarkers()
        trunk_msg.header = msg.header
        trunk_msg.frame_number = msg.frame_number
        trunk_msg.translations = [marker.translation for marker in msg.markers]
        self.publisher.publish(trunk_msg)
        # self.get_logger().info('Published TrunkMarkers message.')

def main(args=None):
    rclpy.init(args=args)
    node = ConverterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
