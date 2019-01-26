from pybulletgym.envs.mujoco.robots.locomotors.walker_base import WalkerBase
from pybulletgym.envs.mujoco.robots.robot_bases import MJCFBasedRobot
import numpy as np


class Ant(WalkerBase, MJCFBasedRobot):
    foot_list = ['front_left_foot', 'front_right_foot', 'left_back_foot', 'right_back_foot']

    def __init__(self):
        WalkerBase.__init__(self, power=2.5)
        MJCFBasedRobot.__init__(self, "ant.xml", "torso", action_dim=8, obs_dim=111)

    def calc_state(self):
        WalkerBase.calc_state(self)
        pose = self.parts['torso'].get_pose()
        qpos = np.hstack((pose, [j.get_position() for j in self.ordered_joints])).flatten()  # shape (15,)

        velocity = self.parts['torso'].get_velocity()
        qvel = np.hstack((velocity[0], velocity[1], [j.get_velocity() for j in self.ordered_joints])).flatten()  # shape (14,)

        cfrc_ext = np.zeros((14, 6))  # shape (14, 6)  # TODO: FIND cfrc_ext
        return np.concatenate([
            qpos.flat[2:],                   # self.sim.data.qpos.flat[2:],
            qvel.flat,						 # self.sim.data.qvel.flat,
            np.clip(cfrc_ext, -1, 1).flat    # np.clip(self.sim.data.cfrc_ext, -1, 1).flat,
        ])

    def alive_bonus(self, z, pitch):
        return +1 if z > 0.26 else -1  # 0.25 is central sphere rad, die if it scrapes the ground
