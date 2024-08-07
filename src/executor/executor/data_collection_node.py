import os
import csv
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from interfaces.msg import SingleMotorControl, AllMotorsControl, TrunkMarkers, TrunkRigidBodies

class DataCollectionNode(Node):
    def __init__(self):
        super().__init__('data_collection_node')
        self.declare_parameters(namespace='', parameters=[
            ('number_of_samples', 10),
            ('update_period', 0.1),  # in [s]
            ('data_type', 'steady_state')  # 'steady_state' or 'dynamic' TODO: implement dynamic trajectory data collection
        ])

        self.sample_size = self.get_parameter('number_of_samples').value
        self.update_period = self.get_parameter('update_period').value
        self.data_type = self.get_parameter('data_type').value
        
        self.is_collecting = False
        self.previous_time = time.time()
        self.current_control_id = -1
        self.control_inputs = None
        self.data_dir = os.getenv('TRUNK_DATA', '/home/asl/Documents/asl_trunk_ws/data')

        self.subscription_markers = self.create_subscription(
            TrunkMarkers,
            '/trunk_markers',
            self.listener_markers_callback,
            QoSProfile(depth=10)
        )

        self.controls_publisher = self.create_publisher(
            AllMotorsControl,
            '/all_motors_control',
            QoSProfile(depth=10)
        )

    def listener_markers_callback(self, msg):
        if not self.is_collecting:
            # Reset and start collecting new marker data
            self.stored_markers = []
            self.check_settled_markers = []
            self.is_collecting = True

            # Publish new motor control data
            self.current_control_id = self.current_control_id + 1
            self.control_inputs = self.get_control_inputs()
            if self.control_inputs == None:
                self.get_logger().info('Data collection has finished.')
                self.destroy_node()
                rclpy.shutdown()
            else:
                control_message = AllMotorsControl()
                list_single_motor_control = []
                for i in range(len(self.control_inputs)):
                    single_motor_control = SingleMotorControl()
                    single_motor_control.mode = 0  # TODO: mode is hardcoded, should be a set parameter as well
                    single_motor_control.value = self.control_inputs[i]
                    list_single_motor_control.append(single_motor_control)
                control_message.motors_control = list_single_motor_control
                self.controls_publisher.publish(control_message)
                self.get_logger().info('Published new motor control setting: ' + str(self.control_inputs))

        if self.is_collecting and (time.time() - self.previous_time) >= self.update_period:
            self.previous_time = time.time()
            if self.check_settled():
                self.stored_markers.append(msg.translations[0])  # TODO: single marker is hardcoded
                if len(self.stored_markers) >= self.sample_size:
                    self.is_collecting = False
                    self.process_data()
            else:
                self.check_settled_markers.append(msg.translations[0])

    def check_settled(self, tolerance=0.005, marker_window=5):
        if len(self.check_settled_markers) < marker_window:
            # Not enough markers to determine if settled
            return False

        min_position = {'x': float('inf'), 'y': float('inf'), 'z': float('inf')}
        max_position = {'x': float('-inf'), 'y': float('-inf'), 'z': float('-inf')}
        recent_markers = self.check_settled_markers[-marker_window:]
        for marker in recent_markers:
            min_position['x'] = min(min_position['x'], marker.x)
            max_position['x'] = max(max_position['x'], marker.x)
            min_position['y'] = min(min_position['y'], marker.y)
            max_position['y'] = max(max_position['y'], marker.y)
            min_position['z'] = min(min_position['z'], marker.z)
            max_position['z'] = max(max_position['z'], marker.z)

        range_x = max_position['x'] - min_position['x']
        range_y = max_position['y'] - min_position['y']
        range_z = max_position['z'] - min_position['z']

        if range_x <= tolerance and range_y <= tolerance and range_z <= tolerance:
            # The markers in the window have settled
            return True
        else:
            # The markers in the window have not settled
            return False

    def get_control_inputs(self):
        control_inputs = None
        control_input_csv_file = os.path.join(self.data_dir, 'trajectories/steady_state/control_inputs.csv')
        with open(control_input_csv_file, mode='r') as file:
            csv_reader = csv.reader(file)
            next(csv_reader, None)  # skip the header row
            for row in csv_reader:
                if int(row[0]) == self.current_control_id:
                    control_inputs = [float(u) for u in row[1:]]
                    break
        return control_inputs

    def process_data(self):
        # Take average position over all stored samples
        average_position = {
            'x': sum(marker.x for marker in self.stored_markers) / self.sample_size,
            'y': sum(marker.y for marker in self.stored_markers) / self.sample_size,
            'z': sum(marker.z for marker in self.stored_markers) / self.sample_size
        }
        
        # Save data to CSV
        trajectory_csv_file = os.path.join(self.data_dir, 'trajectories/steady_state/steady_state_trajectories.csv')
        with open(trajectory_csv_file, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(self.control_inputs + [average_position['x'], average_position['y'], average_position['z']])
        self.get_logger().info('Stored new sample with marker position: ' + str(average_position) + ' [m].')

def main(args=None):
    rclpy.init(args=args)
    data_collection_node = DataCollectionNode()
    rclpy.spin(data_collection_node)
    data_collection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
