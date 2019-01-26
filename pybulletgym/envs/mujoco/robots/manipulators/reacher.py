from pybulletgym.envs.mujoco.robots.robot_bases import MJCFBasedRobot
import numpy as np


class Reacher(MJCFBasedRobot):
    TARG_LIMIT = 0.27

    def __init__(self):
        MJCFBasedRobot.__init__(self, 'reacher.xml', 'body0', action_dim=2, obs_dim=9)

    def robot_specific_reset(self, bullet_client):
        self.jdict["target_x"].reset_current_position(
            self.np_random.uniform(low=-self.TARG_LIMIT, high=self.TARG_LIMIT), 0)
        self.jdict["target_y"].reset_current_position(
            self.np_random.uniform(low=-self.TARG_LIMIT, high=self.TARG_LIMIT), 0)
        self.fingertip = self.parts["fingertip"]
        self.target = self.parts["target"]
        self.central_joint = self.jdict["joint0"]
        self.elbow_joint = self.jdict["joint1"]
        self.central_joint.reset_current_position(self.np_random.uniform(low=-3.14, high=3.14), 0)
        self.elbow_joint.reset_current_position(self.np_random.uniform(low=-3.14, high=3.14), 0)

    def apply_action(self, a):
        assert (np.isfinite(a).all())
        self.central_joint.set_motor_torque(0.05 * float(np.clip(a[0], -1, +1)))
        self.elbow_joint.set_motor_torque(0.05 * float(np.clip(a[1], -1, +1)))

    def calc_state(self):
        target_x, target_vx = self.jdict["target_x"].current_position()
        target_y, target_vy = self.jdict["target_y"].current_position()

        qpos = np.array([j.current_position() for j in self.ordered_joints])  # shape (4,)
        qvel = np.array([j.current_relative_position()[1] for j in self.ordered_joints])  # shape (4,) # TODO: Add target pos and vel

        theta = qpos[:2]
        self.to_target_vec = np.array(self.fingertip.pose().xyz()) - np.array(self.target.pose().xyz())  # shape (3,)

        return np.concatenate([
            np.cos(theta),  # np.cos(theta),
            np.sin(theta),  # np.sin(theta),
            qpos.flat[2:],  # self.sim.data.qpos.flat[2:],
            qvel.flat[:2],  # self.sim.data.qvel.flat[:2],
            self.to_target_vec   # self.get_body_com("fingertip") - self.get_body_com("target")
        ])

    def calc_potential(self):
        return -100 * np.linalg.norm(self.to_target_vec)