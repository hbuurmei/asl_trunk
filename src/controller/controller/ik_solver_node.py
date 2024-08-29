import os
import numpy as np
import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from rclpy.qos import QoSProfile  # type: ignore
from interfaces.msg import SingleMotorControl, AllMotorsControl, TrunkMarkers, TrunkRigidBodies
from interfaces.srv import ControlSolver


class IKSolverNode(Node):
    def __init__(self):
        super().__init__('ik_solver_node')
        self.declare_parameters(namespace='', parameters=[
            ('u2y_file', 'u2y.npy'),
            ('y2u_file', 'y2u.npy'),
            ('u_min', -0.25),
            ('u_max', 0.25),
        ])

        self.u2y_file = self.get_parameter('u2y_file').value
        self.y2u_file = self.get_parameter('y2u_file').value
        self.u_min = self.get_parameter('u_min').value
        self.u_max = self.get_parameter('u_max').value

        # Get mappings
        self.data_dir = os.getenv('TRUNK_DATA', '/home/asl/Documents/asl_trunk_ws/data')
        self.u2y = np.load(os.path.join(self.data_dir, f'models/ik/{self.u2y_file}'))
        self.y2u = np.load(os.path.join(self.data_dir, f'models/ik/{self.y2u_file}'))

        # Define service, which uses the ik callback function
        self.srv = self.create_service(ControlSolver, 'ik_solver', self.ik_callback)
        self.get_logger().info('Control solver service has been created.')


    def ik_callback(self, request, response):
        """
        Callback function that runs when the service is queried.
        Request contains: current_state (can be observations too)
        Response contains: control_action to be taken
        """
        current_state = np.array(request.current_state)
        control_action = np.clip(self.y2u @ current_state, self.u_min, self.u_max)
        response.control_action = control_action.tolist()
        return response


def main(args=None):
    rclpy.init(args=args)
    ik_solver_node = IKSolverNode()
    rclpy.spin(ik_solver_node)
    ik_solver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
