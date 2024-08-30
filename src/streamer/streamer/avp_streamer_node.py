import os
import csv
import time
import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from rclpy.qos import QoSProfile  # type: ignore
from interfaces.msg import TrunkRigidBodies
from geometry_msgs.msg import Point
from .avp_subscriber import AVPSubscriber


class AVPStreamerNode(Node):
    def __init__(self):
        super().__init__('avp_streamer_node')
        self.declare_parameters(namespace='', parameters=[
            ('debug', False),                               # False or True
            ('recording_name', 'test_task_recording')
        ])

        self.debug = self.get_parameter('debug').value
        self.recording_name = self.get_parameter('recording_name').value
        self.data_dir = os.getenv('TRUNK_DATA', '/home/asl/Documents/asl_trunk_ws/data')
        self.recording_file = os.path.join(self.data_dir, f'trajectories/teleop/{self.recording_name}.csv')

        # Initialize stored positions and gripper states
        # self.stored_positions = []
        # self.stored_gripper_states = []

        # Keep track of trajectory ID
        self.recording_id = -1

        self.avp_publisher = self.create_publisher(
            TrunkRigidBodies,
            '/avp_des_positions',
            QoSProfile(depth=10)
        )

        self.streamer = AVPSubscriber(
            ip='10.93.181.122',
            gripper_open=False,
            recording_file=self.recording_file
            )
        
        self._timer = self.create_timer(1.0 / 10.0, self.streamer_data_sampling_callback)
        
    def streamer_data_sampling_callback(self):
        # Determine trajectory ID
        if self.streamer.isRecording and not self.streamer.previousRecordingState:
            self.stored_positions = []
            self.stored_gripper_states = []
            self.recording_id += 1

        avp_positions = self.streamer.get_latest()
        trunk_rigid_bodies_msg = self.convert_avp_positions_to_trunk_rigid_bodies(avp_positions)
        self.avp_publisher.publish(trunk_rigid_bodies_msg)
        if self.debug:
            self.get_logger().info(f'Published AVP desired positions {avp_positions}.')

        # Store trajectory
        if not self.streamer.isRecording and self.streamer.previousRecordingState:
            self.process_data(trunk_rigid_bodies_msg.names)

    def convert_avp_positions_to_trunk_rigid_bodies(self, avp_positions):
        """
        Converts the given AVP positions into a TrunkRigidBodies message.
        """
        msg = TrunkRigidBodies()

        # Extract rigid body names and positions
        rigid_body_names = ["1", "2", "3"]
        positions_list = [
            avp_positions['disk_positions'].get(f'disk{name}', [0, 0, 0]) for name in rigid_body_names
        ]

        msg.frame_number = 0  # TODO: replace?
        msg.rigid_body_names = rigid_body_names
        points = []
        for position in positions_list:
            point = Point()
            point.x = position[0]
            point.y = position[1]
            point.z = position[2]
            points.append(point)
        msg.positions = points

        return msg

    def process_data(self, names):
        # Populate the header row of the CSV file with states if it does not exist
        if not os.path.exists(self.recording_file):
            header = ['ID'] + [f'{axis}{name}' for name in names for axis in ['x', 'y', 'z']] + ['isGripperOpen']
            with open(self.recording_file, 'w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(header)
        
        # Store all positions and gripper state in a CSV file
        with open(self.recording_file, 'a', newline='') as file:
            writer = csv.writer(file)
            for i, pos_list in enumerate(self.recorded_positions):
                row = [self.streamer.recording_id] + [coord for pos in pos_list for coord in [pos.x, pos.y, pos.z]] + [self.stored_gripper_states[i]]
                writer.writerow(row)
        self.get_logger().info(f'Stored the data corresponding to the {self.streamer.recording_id}th trajectory.')


def main(args=None):
    rclpy.init(args=args)
    avp_streamer_node = AVPStreamerNode()
    rclpy.spin(avp_streamer_node)
    avp_streamer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
