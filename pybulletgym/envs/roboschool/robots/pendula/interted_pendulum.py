from pybulletgym.envs.roboschool.robots.robot_bases import MJCFBasedRobot
import numpy as np


class InvertedPendulum(MJCFBasedRobot):
    swingUp = False

    def __init__(self):
        MJCFBasedRobot.__init__(self, 'inverted_pendulum.xml', 'cart', action_dim=1, obs_dim=5)

        self._p = None
        self.pole = None
        self.slider = None
        self.j1 = None
        self.np_random = None
        self.theta = None
        self.obs_joints = []

    def robot_specific_reset(self, bullet_client):
        self._p = bullet_client
        self.pole = self.parts["pole"]
        self.slider = self.jdict["slider"]
        self.obs_joints.append(self.slider)
        self.j1 = self.jdict["hinge"]
        self.obs_joints.append(self.j1)
        u = self.np_random.uniform(low=-.1, high=.1)
        self.j1.reset_current_position(u if not self.swingUp else 3.1415 + u, 0)
        self.j1.set_motor_torque(0)

    def apply_action(self, a):

        assert(np.isfinite(a).all())

        self.slider.set_motor_torque(100 * float(np.clip(a[0], -1, +1)))

    def calc_state(self):
        x, vx = self.slider.current_position()
        self.theta, theta_dot = self.j1.current_position()

        state_array = np.array([
            x, vx,
            np.cos(self.theta), np.sin(self.theta), theta_dot
        ])

        assert(np.isfinite(state_array).all())
        return np.array(state_array)



class InvertedPendulumSwingup(InvertedPendulum):
    swingUp = True
