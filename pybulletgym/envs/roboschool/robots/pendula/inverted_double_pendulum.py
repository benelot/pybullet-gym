from pybulletgym.envs.roboschool.robots.robot_bases import MJCFBasedRobot
import numpy as np


class InvertedDoublePendulum(MJCFBasedRobot):
    def __init__(self):
        MJCFBasedRobot.__init__(self,  'inverted_double_pendulum.xml', 'cart', action_dim=1, obs_dim=9)

    def robot_specific_reset(self, bullet_client):
        self._p = bullet_client
        self.pole2 = self.parts["pole2"]
        self.slider = self.jdict["slider"]
        self.j1 = self.jdict["hinge"]
        self.j2 = self.jdict["hinge2"]
        u = self.np_random.uniform(low=-.1, high=.1, size=[2])
        self.j1.reset_current_position(float(u[0]), 0)
        self.j2.reset_current_position(float(u[1]), 0)
        self.j1.set_motor_torque(0)
        self.j2.set_motor_torque(0)

    def apply_action(self, a):
        assert(np.isfinite(a).all())
        self.slider.set_motor_torque(200*float(np.clip(a[0], -1, +1)))

    def calc_state(self):
        theta, theta_dot = self.j1.current_position()
        gamma, gamma_dot = self.j2.current_position()
        x, vx = self.slider.current_position()
        self.pos_x, _, self.pos_y = self.pole2.pose().xyz()
        assert(np.isfinite(x))
        return np.array([
            x, vx,
            self.pos_x,
            np.cos(theta), np.sin(theta), theta_dot,
            np.cos(gamma), np.sin(gamma), gamma_dot,
        ])
