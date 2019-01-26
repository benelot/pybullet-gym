from pybulletgym.envs.roboschool.robots.locomotors.walker_base import WalkerBase
from pybulletgym.envs.roboschool.robots.robot_bases import MJCFBasedRobot
import numpy as np


class Humanoid(WalkerBase, MJCFBasedRobot):
    self_collision = True
    foot_list = ["right_foot", "left_foot"]  # "left_hand", "right_hand"

    def __init__(self, random_yaw = False, random_lean=False):
        WalkerBase.__init__(self, power=0.41)
        MJCFBasedRobot.__init__(self, 'humanoid_symmetric.xml', 'torso', action_dim=17, obs_dim=44)
        # 17 joints, 4 of them important for walking (hip, knee), others may as well be turned off, 17/4 = 4.25
        self.random_yaw = random_yaw
        self.random_lean = random_lean

    def robot_specific_reset(self, bullet_client):
        WalkerBase.robot_specific_reset(self, bullet_client)
        self.motor_names  = ["abdomen_z", "abdomen_y", "abdomen_x"]
        self.motor_power  = [100, 100, 100]
        self.motor_names += ["right_hip_x", "right_hip_z", "right_hip_y", "right_knee"]
        self.motor_power += [100, 100, 300, 200]
        self.motor_names += ["left_hip_x", "left_hip_z", "left_hip_y", "left_knee"]
        self.motor_power += [100, 100, 300, 200]
        self.motor_names += ["right_shoulder1", "right_shoulder2", "right_elbow"]
        self.motor_power += [75, 75, 75]
        self.motor_names += ["left_shoulder1", "left_shoulder2", "left_elbow"]
        self.motor_power += [75, 75, 75]
        self.motors = [self.jdict[n] for n in self.motor_names]
        if self.random_yaw:
            position = [0,0,0]
            orientation = [0,0,0]
            yaw = self.np_random.uniform(low=-3.14, high=3.14)
            if self.random_lean and self.np_random.randint(2) == 0:
                if self.np_random.randint(2) == 0:
                    pitch = np.pi/2
                    position = [0, 0, 0.45]
                else:
                    pitch = np.pi*3/2
                    position = [0, 0, 0.25]
                roll = 0
                orientation = [roll, pitch, yaw]
            else:
                position = [0, 0, 1.4]
                orientation = [0, 0, yaw]  # just face random direction, but stay straight otherwise
            self.robot_body.reset_position(position)
            self.robot_body.reset_orientation(p.getQuaternionFromEuler(orientation))
        self.initial_z = 0.8

    def apply_action(self, a):
        assert(np.isfinite(a).all())
        force_gain = 1
        for i, m, power in zip(range(17), self.motors, self.motor_power):
            m.set_motor_torque(float(force_gain * power * self.power * np.clip(a[i], -1, +1)))

    def alive_bonus(self, z, pitch):
        return +2 if z > 0.78 else -1   # 2 here because 17 joints produce a lot of electricity cost just from policy noise, living must be better than dying
