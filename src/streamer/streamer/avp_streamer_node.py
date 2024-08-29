import os
import csv
import time
import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from rclpy.qos import QoSProfile  # type: ignore
from interfaces.msg import TrunkMarkers, TrunkRigidBodies


class AVPStreamerNode(Node):
    def __init__(self):
        super().__init__('avp_streamer_node')
        self.declare_parameters(namespace='', parameters=[
            ('recording_name', 'test_task_recording')
        ])


def main(args=None):
    rclpy.init(args=args)
    avp_streamer_node = AVPStreamerNode()
    rclpy.spin(avp_streamer_node)
    avp_streamer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
