from pybulletgym.envs.mujoco.robots.locomotors.walker_base import WalkerBase
from pybulletgym.envs.mujoco.robots.robot_bases import MJCFBasedRobot
import numpy as np


class Hopper(WalkerBase, MJCFBasedRobot):
    """
    Hopper implementation based on MuJoCo.
    """
    foot_list = ["foot"]

    def __init__(self):
        WalkerBase.__init__(self, power=0.75)
        MJCFBasedRobot.__init__(self, "hopper.xml", "torso", action_dim=3, obs_dim=11, add_ignored_joints=True)

        self.pos_after = 0

    def calc_state(self):
        qpos = np.array([j.get_position() for j in self.ordered_joints], dtype=np.float32).flatten()  # shape (6,)
        qvel = np.array([j.get_velocity() for j in self.ordered_joints], dtype=np.float32).flatten()  # shape (6,)

        return np.concatenate([
            qpos.flat[1:],                   # self.sim.data.qpos.flat[1:],
            np.clip(qvel, -10, 10).flat		# self.sim.data.qvel.flat,
        ])

    def calc_potential(self):
        # progress in potential field is speed*dt, typical speed is about 2-3 meter per second, this potential will change 2-3 per frame (not per second),
        # all rewards have rew/frame units and close to 1.0
        pos_before = self.pos_after
        self.pos_after = self.robot_body.get_pose()[0]
        debugmode = 0
        if debugmode:
            print("calc_potential: self.walk_target_dist")
            print(self.walk_target_dist)
            print("self.scene.dt")
            print(self.scene.dt)
            print("self.scene.frame_skip")
            print(self.scene.frame_skip)
            print("self.scene.timestep")
            print(self.scene.timestep)
        return (self.pos_after-pos_before) / self.scene.dt
