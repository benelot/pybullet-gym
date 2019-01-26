from pybulletgym.envs.roboschool.envs.locomotion.humanoid_env import HumanoidBulletEnv
from pybulletgym.envs.roboschool.robots.locomotors import HumanoidFlagrun, HumanoidFlagrunHarder


class HumanoidFlagrunBulletEnv(HumanoidBulletEnv):
    random_yaw = True

    def __init__(self):
        self.robot = HumanoidFlagrun()
        HumanoidBulletEnv.__init__(self, self.robot)

    def create_single_player_scene(self, bullet_client):
        s = HumanoidBulletEnv.create_single_player_scene(self, bullet_client)
        s.zero_at_running_strip_start_line = False
        return s


class HumanoidFlagrunHarderBulletEnv(HumanoidBulletEnv):
    random_lean = True  # can fall on start

    def __init__(self):
        self.robot = HumanoidFlagrunHarder()
        self.electricity_cost /= 4   # don't care that much about electricity, just stand up!
        HumanoidBulletEnv.__init__(self, self.robot)

    def create_single_player_scene(self, bullet_client):
        s = HumanoidBulletEnv.create_single_player_scene(self, bullet_client)
        s.zero_at_running_strip_start_line = False
        return s
