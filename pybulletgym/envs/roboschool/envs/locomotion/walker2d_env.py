from pybulletgym.envs.roboschool.envs.locomotion.walker_base_env import WalkerBaseBulletEnv
from pybulletgym.envs.roboschool.robots.locomotors import Walker2D


class Walker2DBulletEnv(WalkerBaseBulletEnv):
    def __init__(self):
        self.robot = Walker2D()
        WalkerBaseBulletEnv.__init__(self, self.robot)

