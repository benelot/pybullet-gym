from pybulletgym.envs.roboschool.robots.locomotors.walker_base import WalkerBase
from pybulletgym.envs.roboschool.robots.robot_bases import URDFBasedRobot
import numpy as np
import pybullet as p


class Atlas(WalkerBase, URDFBasedRobot):
    random_yaw = False
    foot_list = ["r_foot", "l_foot"]

    def __init__(self):
        WalkerBase.__init__(self, power=2.9)
        URDFBasedRobot.__init__(self, "atlas/atlas_description/atlas_v4_with_multisense.urdf", "pelvis", action_dim=30, obs_dim=70)

    def alive_bonus(self, z, pitch):
        # This is debug code to fix unwanted self-collisions:
        #for part in self.parts.values():
        #	contact_names = set(x.name for x in part.contact_list())
        #	if contact_names:
        #		print("CONTACT OF '%s' WITH '%s'" % (part.name, ",".join(contact_names)) )

        x, y, z = self.head.pose().xyz()
        # Failure mode: robot doesn't bend knees, tries to walk using hips.
        # We fix that by a bit of reward engineering.
        knees = np.array([j.current_relative_position() for j in [self.jdict["l_leg_kny"], self.jdict["r_leg_kny"]]], dtype=np.float32).flatten()
        knees_at_limit = np.count_nonzero(np.abs(knees[0::2]) > 0.99)
        return +4-knees_at_limit if z > 1.3 else -1

    def robot_specific_reset(self, bullet_client):
        WalkerBase.robot_specific_reset(self, bullet_client)
        self.set_initial_orientation(yaw_center=0, yaw_random_spread=np.pi)
        self.head = self.parts["head"]

    def set_initial_orientation(self, yaw_center, yaw_random_spread):
        if not self.random_yaw:
            yaw = yaw_center
        else:
            yaw = yaw_center + self.np_random.uniform(low=-yaw_random_spread, high=yaw_random_spread)

        position = [self.start_pos_x, self.start_pos_y, self.start_pos_z + 1.0]
        orientation = [0, 0, yaw]  # just face random direction, but stay straight otherwise
        self.robot_body.reset_pose(position, p.getQuaternionFromEuler(orientation))
        self.initial_z = 1.5
