from pybulletgym.envs.roboschool.envs.locomotion.walker_base_env import WalkerBaseBulletEnv
from pybulletgym.envs.roboschool.robots.locomotors import Hopper


class HopperBulletEnv(WalkerBaseBulletEnv):
    def __init__(self):
        self.robot = Hopper()
        WalkerBaseBulletEnv.__init__(self, self.robot)

