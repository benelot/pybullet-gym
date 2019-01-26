from pybulletgym.envs.roboschool.robots.locomotors.walker_base import WalkerBase
from pybulletgym.envs.roboschool.robots.robot_bases import MJCFBasedRobot


class Hopper(WalkerBase, MJCFBasedRobot):
    foot_list = ["foot"]

    def __init__(self):
        WalkerBase.__init__(self, power=0.75)
        MJCFBasedRobot.__init__(self, "hopper.xml", "torso", action_dim=3, obs_dim=15)

    def alive_bonus(self, z, pitch):
        return +1 if z > 0.8 and abs(pitch) < 1.0 else -1
