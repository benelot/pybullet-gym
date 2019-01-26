from pybulletgym.envs.mujoco.robots.locomotors.walker_base import WalkerBase
from pybulletgym.envs.mujoco.robots.robot_bases import MJCFBasedRobot
import numpy as np


class HalfCheetah(WalkerBase, MJCFBasedRobot):
    """
    Half Cheetah implementation based on MuJoCo.
    """
    foot_list = ["ffoot", "fshin", "fthigh",  "bfoot", "bshin", "bthigh"]  # track these contacts with ground

    def __init__(self):
        WalkerBase.__init__(self, power=1)
        MJCFBasedRobot.__init__(self, "half_cheetah.xml", "torso", action_dim=6, obs_dim=17, add_ignored_joints=True)

        self.pos_after = 0

    def calc_state(self):
        qpos = np.array([j.get_position() for j in self.ordered_joints], dtype=np.float32).flatten()  # shape (9,)
        qvel = np.array([j.get_velocity() for j in self.ordered_joints], dtype=np.float32).flatten()  # shape (9,)

        return np.concatenate([
            qpos.flat[1:],           # self.sim.data.qpos.flat[1:],
            qvel.flat		         # self.sim.data.qvel.flat,
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

    def robot_specific_reset(self, bullet_client):
        WalkerBase.robot_specific_reset(self, bullet_client)
        for part_id, part in self.parts.items():
            self._p.changeDynamics(part.bodyIndex, part.bodyPartIndex, lateralFriction=0.8, spinningFriction=0.1, rollingFriction=0.1, restitution=0.5)

        self.jdict["bthigh"].power_coef = 120.0
        self.jdict["bshin"].power_coef  = 90.0
        self.jdict["bfoot"].power_coef  = 60.0
        self.jdict["fthigh"].power_coef = 140.0
        self.jdict["fshin"].power_coef  = 60.0
        self.jdict["ffoot"].power_coef  = 30.0
