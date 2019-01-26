from pybulletgym.envs.mujoco.envs.locomotion.walker_base_env import WalkerBaseMuJoCoEnv
from pybulletgym.envs.mujoco.robots.locomotors.ant import Ant


class AntMuJoCoEnv(WalkerBaseMuJoCoEnv):
    def __init__(self):
        self.robot = Ant()
        WalkerBaseMuJoCoEnv.__init__(self, self.robot)
