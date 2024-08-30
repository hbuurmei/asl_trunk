import os
import numpy as np
import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from rclpy.qos import QoSProfile  # type: ignore
from interfaces.msg import SingleMotorControl, AllMotorsControl, TrunkMarkers, TrunkRigidBodies
from interfaces.srv import ControlSolver


class MPCSolverNode(Node):
    def __init__(self):
        super().__init__('mpc_solver_node')

        # Define service, which uses the mpc callback function
        self.srv = self.create_service(ControlSolver, 'mpc_solver', self.mpc_callback)


    def mpc_callback(self, request, response):
        """
        Callback function that runs when the service is queried.
        Request contains: current_state (can be observations too)
        Response contains: control_action to be taken
        """
        raise NotImplementedError('MPC is yet to be implemented.')
        # return response


def main(args=None):
    rclpy.init(args=args)
    mpc_solver_node = MPCSolverNode()
    rclpy.spin(mpc_solver_node)
    mpc_solver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
