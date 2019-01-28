from pybulletgym.envs.roboschool.envs.env_bases import BaseBulletEnv
from pybulletgym.envs.roboschool.robots.pendula.interted_pendulum import InvertedPendulum, InvertedPendulumSwingup
from pybulletgym.envs.roboschool.scenes.scene_bases import SingleRobotEmptyScene
import pybulletgym.envs.roboschool.rewards.inverse_pendulum_rewards as inverse_pendulum_rewards
import numpy as np


class InvertedPendulumBulletEnv(BaseBulletEnv):
    def __init__(self):
        self.robot = InvertedPendulum()
        BaseBulletEnv.__init__(self, self.robot)
        self.stateId = -1
        self.reward_funcs = []
        self.reward_funcs.append(inverse_pendulum_rewards.calc_reward)

    def create_single_player_scene(self, bullet_client):
        return SingleRobotEmptyScene(bullet_client, gravity=9.8, timestep=0.0165, frame_skip=1)

    def reset(self):
        if self.stateId >= 0:
            # print("InvertedPendulumBulletEnv reset p.restoreState(",self.stateId,")")
            self._p.restoreState(self.stateId)
        r = BaseBulletEnv._reset(self)
        if self.stateId < 0:
            self.stateId = self._p.saveState()
        # print("InvertedPendulumBulletEnv reset self.stateId=",self.stateId)
        return r

    def step(self, a):
        self.robot.apply_action(a)
        self.scene.global_step()

        reward = 0
        done = False
        state = self.robot.calc_state()
        for f in self.reward_funcs:
            f_reward, f_done = f(self.robot)
            reward += f_reward
            done |= f_done

        self.rewards = [float(reward)]
        self.HUD(state, a, done)
        return state, sum(self.rewards), done, {}

    def camera_adjust(self):
        self.camera.move_and_look_at(0, 1.2, 1.0, 0, 0, 0.5)


class InvertedPendulumSwingupBulletEnv(InvertedPendulumBulletEnv):
    def __init__(self):
        self.robot = InvertedPendulumSwingup()
        BaseBulletEnv.__init__(self, self.robot)
        self.stateId = -1
        self.reward_funcs = []
        self.reward_funcs.append(inverse_pendulum_rewards.calc_reward)
