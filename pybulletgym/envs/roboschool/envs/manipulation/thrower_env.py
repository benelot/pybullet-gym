from pybulletgym.envs.roboschool.envs.env_bases import BaseBulletEnv
from pybulletgym.envs.roboschool.robots.manipulators.thrower import Thrower
from pybulletgym.envs.roboschool.scenes.scene_bases import SingleRobotEmptyScene
import numpy as np


class ThrowerBulletEnv(BaseBulletEnv):
    def __init__(self):
        self.robot = Thrower()
        BaseBulletEnv.__init__(self, self.robot)

    def create_single_player_scene(self, bullet_client):
        return SingleRobotEmptyScene(bullet_client, gravity=0.0, timestep=0.0020, frame_skip=5)

    def step(self, a):
        self.robot.apply_action(a)
        self.scene.global_step()
        state = self.robot.calc_state()  # sets self.to_target_vec

        potential_old = self.potential
        self.potential = self.robot.calc_potential()

        joint_vel = np.array([
            self.robot.shoulder_pan_joint.get_velocity(),
            self.robot.shoulder_lift_joint.get_velocity(),
            self.robot.upper_arm_roll_joint.get_velocity(),
            self.robot.elbow_flex_joint.get_velocity(),
            self.robot.forearm_roll_joint.get_velocity(),
            self.robot.wrist_flex_joint.get_velocity(),
            self.robot.wrist_roll_joint.get_velocity()
        ])

        action_product = np.matmul(np.abs(a), np.abs(joint_vel))
        action_sum = np.sum(a)

        electricity_cost = (
                -0.10 * action_product  # work torque*angular_velocity
                - 0.01 * action_sum  # stall torque require some energy
        )

        stuck_joint_cost = 0
        for j in self.robot.ordered_joints:
            if np.abs(j.current_relative_position()[0]) - 1 < 0.01:
                stuck_joint_cost += -0.1

        object_xy = self.robot.object.pose().xyz()[:2]
        target_xy = self.robot.target.pose().xyz()[:2]

        if not self.robot._object_hit_ground and self.robot.object.pose().xyz()[2] < -0.25:						# TODO: Should the object and target really belong to the robot? Maybe split this off
            self.robot._object_hit_ground = True
            self.robot._object_hit_location = self.robot.object.pose().xyz()

        if self.robot._object_hit_ground:
            object_hit_xy = self.robot._object_hit_location[:2]
            reward_dist = -np.linalg.norm(object_hit_xy - target_xy)
        else:
            reward_dist = -np.linalg.norm(object_xy - target_xy)
        reward_ctrl = - np.square(a).sum()

        self.rewards = [float(self.potential - potential_old), float(electricity_cost), float(stuck_joint_cost),
                        reward_dist, 0.002 * reward_ctrl]
        self.HUD(state, a, False)
        return state, sum(self.rewards), False, {}

    def camera_adjust(self):
        x, y, z = self.robot.fingertip.pose().xyz()
        x *= 0.5
        y *= 0.5
        self.camera.move_and_look_at(0.3, 0.3, 0.3, x, y, z)

