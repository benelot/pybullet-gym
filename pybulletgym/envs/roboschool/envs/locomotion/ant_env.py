from pybulletgym.envs.roboschool.envs.locomotion.walker_base_env import WalkerBaseBulletEnv
from pybulletgym.envs.roboschool.robots.locomotors import Ant


class AntBulletEnv(WalkerBaseBulletEnv):
    def __init__(self):
        self.robot = Ant()
        WalkerBaseBulletEnv.__init__(self, self.robot)
