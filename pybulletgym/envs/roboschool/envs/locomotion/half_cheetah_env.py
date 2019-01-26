from pybulletgym.envs.roboschool.envs.locomotion.walker_base_env import WalkerBaseBulletEnv
from pybulletgym.envs.roboschool.robots.locomotors import HalfCheetah


class HalfCheetahBulletEnv(WalkerBaseBulletEnv):
    def __init__(self):
        self.robot = HalfCheetah()
        WalkerBaseBulletEnv.__init__(self, self.robot)
