import numpy as np
import pandas as pd

from double_pendulum.controller.abstract_controller import AbstractController
from double_pendulum.model.plant import DoublePendulumPlant


class FeedforwardController(AbstractController):
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
        csv_path="",
    ):
        self.mass = mass
        self.length = length
        self.com = com
        self.damping = damping
        self.cfric = coulomb_fric
        self.gravity = gravity
        self.inertia = inertia
        self.torque_limit = torque_limit

        self.plant = DoublePendulumPlant(
            mass=self.mass,
            length=self.length,
            com=self.com,
            damping=self.damping,
            gravity=self.gravity,
            coulomb_fric=self.cfric,
            inertia=self.inertia,
            torque_limit=self.torque_limit,
        )

        data = pd.read_csv(csv_path)
        time_traj = np.asarray(data["time"])
        pos1_traj = np.asarray(data["pos1"])
        pos2_traj = np.asarray(data["pos2"])
        vel1_traj = np.asarray(data["vel1"])
        vel2_traj = np.asarray(data["vel2"])
        acc1_traj = np.asarray(data["acc1"])
        acc2_traj = np.asarray(data["acc2"])

        self.T = time_traj.T
        self.X = np.asarray([pos1_traj, pos2_traj, vel1_traj, vel2_traj]).T
        self.ACC = np.asarray([acc1_traj, acc2_traj]).T

        self.counter = 0

    def init(self):
        self.counter = 0

    def get_control_output(self, x, t=None):
        u = self.plant.inverse_dynamics(self.X[self.counter], self.ACC[self.counter])

        u[0] = np.clip(u[0], -self.torque_limit[0], self.torque_limit[0])
        u[1] = np.clip(u[1], -self.torque_limit[1], self.torque_limit[1])

        self.counter += 1
        return u

    def get_init_trajectory(self):
        return self.T, self.X, None
