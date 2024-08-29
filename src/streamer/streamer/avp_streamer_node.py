import os
import csv
import time
import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from rclpy.qos import QoSProfile  # type: ignore
from interfaces.msg import TrunkRigidBodies


class AVPStreamerNode(Node):
    def __init__(self):
        super().__init__('avp_streamer_node')
        self.declare_parameters(namespace='', parameters=[
            ('recording_name', 'test_task_recording')
        ])

        self.recording_name = self.get_parameter('recording_name').value

        self.avp_publisher = self.create_publisher(
            TrunkRigidBodies,
            '/avp_des_positions',
            QoSProfile(depth=10)
        )


def main(args=None):
    rclpy.init(args=args)
    avp_streamer_node = AVPStreamerNode()
    rclpy.spin(avp_streamer_node)
    avp_streamer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
