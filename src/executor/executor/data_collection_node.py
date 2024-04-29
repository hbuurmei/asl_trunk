import csv
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from interfaces.msg import AllMotorControl, TrunkMarkers

class DataCollectionNode(Node):
    def __init__(self):
        super().__init__('data_collection_node')
        self.declare_parameters(namespace='', parameters=[
            ('number_of_samples', 10),
            ('update_period', 0.1),  # in [s]
        ])

        self.sample_size = self.get_parameter('number_of_samples').value
        self.stored_markers = []
        self.check_settled_markers = []
        self.is_collecting = False
        self.update_period = self.get_parameter('update_period').value
        self.previous_time = time.time()

        self.subscription_markers = self.create_subscription(
            TrunkMarkers,
            '/trunk_markers',
            self.listener_markers_callback,
            QoSProfile(depth=10)
        )

        self.publisher_ = self.create_publisher(
            AllMotorControl,
            '/all_motors_control',
            QoSProfile(depth=10)
        )

    def listener_markers_callback(self, msg):
        if not self.is_collecting:
            # Reset and start collecting new marker data
            self.stored_markers = []
            self.check_settled_markers = []
            self.is_collecting = True

        if self.is_collecting and (time.time() - self.previous_time) >= self.update_period:
            if self.check_settled():
                self.stored_markers.append(msg.translations[0])  # TODO: single marker is hardcoded
                if len(self.stored_markers) >= self.sample_size:
                    self.is_collecting = False
                    self.process_data()
            else:
                self.check_settled_markers.append(msg.translations[0])

    def check_settled(self):
        # Check if the marker has settled
        return False

    def process_data(self):

        # Save data to CSV
        with open('../../data/steady_state_data.csv', 'a', newline='') as file:
            writer = csv.writer(file)
            # writer.writerow([average_position['x'], average_position['y'], average_position['z'], control_message.control_input1, control_message.control_input2])

        # Publish new motor control data
        control_message = AllMotorControl()
        self.publisher_.publish(control_message)
        self.get_logger().info('Published new motor control setting.')

def main(args=None):
    rclpy.init(args=args)
    data_collection_node = DataCollectionNode()
    rclpy.spin(data_collection_node)
    data_collection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
