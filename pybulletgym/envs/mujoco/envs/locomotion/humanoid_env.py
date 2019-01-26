from pybulletgym.envs.mujoco.envs.locomotion.walker_base_env import WalkerBaseMuJoCoEnv
from pybulletgym.envs.mujoco.robots.locomotors.humanoid import Humanoid


class HumanoidMuJoCoEnv(WalkerBaseMuJoCoEnv):
    def __init__(self, robot=Humanoid()):
        self.robot = robot
        WalkerBaseMuJoCoEnv.__init__(self, self.robot)
        self.electricity_cost  = 4.25 * WalkerBaseMuJoCoEnv.electricity_cost
        self.stall_torque_cost = 4.25 * WalkerBaseMuJoCoEnv.stall_torque_cost
