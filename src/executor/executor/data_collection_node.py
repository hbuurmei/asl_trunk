import os
import csv
import time
import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from rclpy.qos import QoSProfile  # type: ignore
from interfaces.msg import SingleMotorControl, AllMotorsControl, TrunkMarkers, TrunkRigidBodies

def load_control_inputs(control_input_csv_file):
    control_inputs_dict = {}
    with open(control_input_csv_file, mode='r') as file:
        csv_reader = csv.reader(file)
        next(csv_reader, None)  # skip the header row
        for row in csv_reader:
            control_id = int(row[0])
            control_inputs = [float(u) for u in row[1:]]
            control_inputs_dict[control_id] = control_inputs
    return control_inputs_dict

class DataCollectionNode(Node):
    def __init__(self):
        super().__init__('data_collection_node')
        self.declare_parameters(namespace='', parameters=[
            ('number_of_samples', 10),
            ('update_period', 0.1),  # in [s]
            ('data_type', 'steady_state'),  # 'steady_state' or 'dynamic' TODO: implement dynamic trajectory data collection
            ('mocap_type', 'rigid_bodies')  # 'rigid_bodies' or 'markers'
            ('control_type', 'output')  # 'output' or 'position'
        ])

        self.sample_size = self.get_parameter('number_of_samples').value
        self.update_period = self.get_parameter('update_period').value
        self.data_type = self.get_parameter('data_type').value
        self.mocap_type = self.get_parameter('mocap_type').value
        self.control_type = self.get_parameter('control_type').value
        
        self.is_collecting = False
        self.previous_time = time.time()
        self.current_control_id = -1
        self.control_inputs = None
        self.data_dir = os.getenv('TRUNK_DATA', '/home/asl/Documents/asl_trunk_ws/data')

        control_input_csv_file = os.path.join(self.data_dir, 'trajectories/steady_state/control_inputs.csv')
        self.control_inputs_dict = load_control_inputs(control_input_csv_file)

        if self.mocap_type == 'markers':
            self.subscription_markers = self.create_subscription(
                TrunkMarkers,
                '/trunk_markers',
                self.listener_callback,
                QoSProfile(depth=10)
            )
        elif self.mocap_type == 'rigid_bodies':
            self.subscription_rigid_bodies = self.create_subscription(
                TrunkRigidBodies,
                '/trunk_rigid_bodies',
                self.listener_callback,
                QoSProfile(depth=10)
            )
        else:
            raise ValueError('Invalid mocap type: ' + self.mocap_type + '. Valid options are: "rigid_bodies" or "markers".')

        self.controls_publisher = self.create_publisher(
            AllMotorsControl,
            '/all_motors_control',
            QoSProfile(depth=10)
        )

    def listener_callback(self, msg):
        if not self.is_collecting:
            # Reset and start collecting new mocap data
            self.stored_positions = []
            self.check_settled_positions = []
            self.is_collecting = True

            # Publish new motor control data
            self.current_control_id += 1
            self.control_inputs = self.control_inputs_dict.get(self.current_control_id)
            if self.control_inputs is None:
                self.get_logger().info('Data collection has finished.')
                self.destroy_node()
                rclpy.shutdown()
            else:
                self.publish_control_inputs()

        if self.is_collecting and (time.time() - self.previous_time) >= self.update_period:
            self.previous_time = time.time()
            if self.check_settled():
                # Store positions
                self.store_positions(msg)
                if len(self.stored_positions) >= self.sample_size:
                    # Data collection is complete and ready to be processed
                    self.is_collecting = False
                    names = self.extract_names(msg)
                    self.process_data(names)
            else:
                self.check_settled_positions.append(self.extract_positions(msg))

    def publish_control_inputs(self):
        control_message = AllMotorsControl()
        mode = 1 if self.control_type == 'position' else 0  # default to 'output' control
        control_message.motors_control = [
            SingleMotorControl(mode=mode, value=value) for value in self.control_inputs
        ]
        self.controls_publisher.publish(control_message)
        self.get_logger().info('Published new motor control setting: ' + str(self.control_inputs))

    def extract_positions(self, msg):
        if self.mocap_type == 'markers':
            return msg.translations
        elif self.mocap_type == 'rigid_bodies':
            return msg.positions
        
    def extract_names(self, msg):
        if self.mocap_type == 'markers':
            raise NotImplementedError('Extracting names from markers is not implemented.')
        elif self.mocap_type == 'rigid_bodies': 
            return msg.rigid_body_names

    def store_positions(self, msg):
        self.stored_positions.append(self.extract_positions(msg))

    def check_settled(self, tolerance=0.005, window=5):
        if len(self.check_settled_positions) < window:
            # Not enough positions to determine if settled
            return False

        min_position = {'x': float('inf'), 'y': float('inf'), 'z': float('inf')}
        max_position = {'x': float('-inf'), 'y': float('-inf'), 'z': float('-inf')}
        recent_positions = self.check_settled_positions[-window:]
        for pos_list in recent_positions:
            for pos in pos_list:
                min_position['x'] = min(min_position['x'], pos.x)
                max_position['x'] = max(max_position['x'], pos.x)
                min_position['y'] = min(min_position['y'], pos.y)
                max_position['y'] = max(max_position['y'], pos.y)
                min_position['z'] = min(min_position['z'], pos.z)
                max_position['z'] = max(max_position['z'], pos.z)

        range_x = max_position['x'] - min_position['x']
        range_y = max_position['y'] - min_position['y']
        range_z = max_position['z'] - min_position['z']

        return range_x <= tolerance and range_y <= tolerance and range_z <= tolerance


    def process_data(self, names):
        # Take average positions over all stored samples
        average_positions = [
            sum(coords) / len(self.stored_positions)
            for pos_list in zip(*self.stored_positions)
            for coords in zip(*[(pos.x, pos.y, pos.z) for pos in pos_list])
        ]

        # Populate the header row of the CSV file with states if it does not exist
        trajectory_csv_file = os.path.join(self.data_dir, 'trajectories/steady_state/steady_state_trajectories.csv')
        if not os.path.exists(trajectory_csv_file):
            header = [f'{axis}{name}' for name in names for axis in ['x', 'y', 'z']]
            with open(trajectory_csv_file, 'w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(header)

        # Save data to CSV
        with open(trajectory_csv_file, 'a', newline='') as file:
            writer = csv.writer(file)            
            writer.writerow(average_positions)
        self.get_logger().info('Stored new sample with positions: ' + str(average_positions) + ' [m].')


def main(args=None):
    rclpy.init(args=args)
    data_collection_node = DataCollectionNode()
    rclpy.spin(data_collection_node)
    data_collection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
