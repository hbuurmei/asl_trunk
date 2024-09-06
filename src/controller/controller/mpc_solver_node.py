import os
import numpy as np
import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from rclpy.qos import QoSProfile  # type: ignore
from interfaces.msg import SingleMotorControl, AllMotorsControl, TrunkMarkers, TrunkRigidBodies
from interfaces.srv import ControlSolver
from controller.mpc.gusto import GuSTO


class MPCSolverNode(Node):
    """
    Defines a service provider node that will run the GuSTO MPC implementation.
    """

    def __init__(self, model, N, dt, Qz, R, x0, t=None, z=None, u=None, Qzf=None, zf=None,
                 U=None, X=None, Xf=None, dU=None, verbose=0, warm_start=True, **kwargs):
        self.model = model
        self.N = N
        self.dt = dt

        # Get characteristic values for GuSTO scaling
        x_char, f_char = self.model.get_characteristic_vals()

        # Define cost function matrices
        self.Qzf = Qzf

        # Define target values
        self.t = t
        self.z = z
        self.u = u
        if z is not None and z.ndim == 2:
            self.z_interp = interp1d(t, z, axis=0,
                                     bounds_error=False, fill_value=(z[0, :], z[-1, :]))

        if u is not None and u.ndim == 2:
            self.u_interp = interp1d(t, u, axis=0,
                                     bounds_error=False, fill_value=(u[0, :], u[-1, :]))

        # Set up GuSTO and run first solve with a simple initial guess
        u_init = np.zeros((self.N, self.model.n_u))
        x_init, _ = self.model.rollout(x0, u_init, self.dt)
        z, zf, u = self.get_target(0.0)
        self.gusto = GuSTO(model, N, dt, Qz, R, x0, u_init, x_init, z=z, u=u,
                           Qzf=Qzf, zf=zf, U=U, X=X, Xf=Xf, dU=dU,
                           verbose=verbose, warm_start=warm_start,
                           x_char=x_char, f_char=f_char, **kwargs)
        self.xopt, self.uopt, _, _ = self.gusto.get_solution()
        self.topt = self.dt * np.arange(self.N + 1)

        # Initialize the ROS node
        super().__init__('mpc_solver_node')

        # Define the service, which uses the gusto callback function
        self.srv = self.create_service(ControlSolver, 'mpc_solver', self.gusto_callback)

    def gusto_callback(self, request, response):
        """
        Callback function that runs when the service is queried, request message contains:
        t0, x0

        and the response message will contain:

        t, xopt, uopt, zopt
        """
        t0 = request.t0
        x0 = arr2np(request.x0, self.model.n_x, squeeze=True)

        # Get target values at proper times by interpolating
        z, zf, u = self.get_target(t0)

        # Get initial guess
        idx0 = np.argwhere(self.topt >= t0)[0, 0]
        u_init = self.uopt[-1, :].reshape(1, -1).repeat(self.N, axis=0)
        u_init[0:self.N - idx0] = self.uopt[idx0:, :]
        x_init = self.xopt[-1, :].reshape(1, -1).repeat(self.N + 1, axis=0)
        x_init[0:self.N + 1 - idx0] = self.xopt[idx0:, :]

        # Solve GuSTO and get solution
        self.gusto.solve(x0, u_init, x_init, z=z, zf=zf, u=u)
        self.xopt, self.uopt, zopt, t_solve = self.gusto.get_solution()

        self.topt = t0 + self.dt * np.arange(self.N + 1)
        response.t = np2arr(self.topt)
        response.xopt = np2arr(self.xopt)
        response.uopt = np2arr(self.uopt)
        response.zopt = np2arr(zopt)
        response.solve_time = t_solve

        return response

    def get_target(self, t0):
        """
        Returns z, zf, u arrays for GuSTO solve
        """
        t = t0 + self.dt * np.arange(self.N + 1)

        # Get target z terms for cost function
        if self.z is not None:
            if self.z.ndim == 2:
                z = self.z_interp(t)
            else:
                z = self.z.reshape(1, -1).repeat(self.N + 1)
        else:
            z = None

        # Get target zf term for cost function
        if self.Qzf is not None and z is not None:
            zf = z[-1, :]
        else:
            zf = None

        # Get target u terms for cost function
        if self.u is not None:
            if self.u.ndim == 2:
                u = self.u_interp(t)
            else:
                u = self.u.reshape(1, -1).repeat(self.N)
        else:
            u = None

        return z, zf, u


def main(args=None):
    rclpy.init(args=args)
    mpc_solver_node = MPCSolverNode()
    rclpy.spin(mpc_solver_node)
    mpc_solver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
