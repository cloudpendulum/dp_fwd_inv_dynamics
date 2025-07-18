import numpy as np

from double_pendulum.controller.abstract_controller import AbstractController
from double_pendulum.controller.gravity_compensation.gravity_compensation_controller import (
    GravityCompensationController,
)
from double_pendulum.controller.pid.point_pid_controller import PointPIDController


class PIDGravityCompensationController(AbstractController):
    def __init__(
        self,
        mass=[0.5, 0.6],
        length=[0.3, 0.2],
        com=[0.3, 0.2],
        damping=[0.1, 0.1],
        coulomb_fric=[0.0, 0.0],
        gravity=9.81,
        inertia=[None, None],
        torque_limit=[0.0, 1.0],
        dt=0.01,
    ):
        self.torque_limit = torque_limit

        self.grav_con = GravityCompensationController(
            mass=mass,
            length=length,
            com=com,
            damping=damping,
            coulomb_fric=coulomb_fric,
            gravity=gravity,
            inertia=inertia,
            torque_limit=torque_limit,
        )

        self.pid_con = PointPIDController(torque_limit=self.torque_limit, dt=dt)

    def set_goal(self, x):
        self.pid_con.set_goal(x)

    def set_parameters(self, Kp, Ki, Kd):
        self.pid_con.set_parameters(Kp, Ki, Kd)

    def init(self):
        self.grav_con.init()
        self.pid_con.init()

    def get_control_output(self, x, t=None):
        grav_u = self.grav_con.get_control_output(x, t)
        pid_u = self.pid_con.get_control_output(x, t)

        u = grav_u + pid_u

        u[0] = np.clip(u[0], -self.torque_limit[0], self.torque_limit[0])
        u[1] = np.clip(u[1], -self.torque_limit[1], self.torque_limit[1])

        return u
