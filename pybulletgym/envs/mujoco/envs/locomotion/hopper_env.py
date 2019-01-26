from pybulletgym.envs.mujoco.envs.locomotion.walker_base_env import WalkerBaseMuJoCoEnv
from pybulletgym.envs.roboschool.robots.locomotors import Hopper
import numpy as np


class HopperMuJoCoEnv(WalkerBaseMuJoCoEnv):
    def __init__(self):
        self.robot = Hopper()
        WalkerBaseMuJoCoEnv.__init__(self, self.robot)

    def step(self, a):
        if not self.scene.multiplayer:  # if multiplayer, action first applied to all robots, then global step() called, then _step() for all robots with the same actions
            self.robot.apply_action(a)
            self.scene.global_step()

        alive_bonus = 1.0
        potential = self.robot.calc_potential()
        power_cost = -1e-3 * np.square(a).sum()
        state = self.robot.calc_state()

        height, ang = state[0], state[1]

        done = not (np.isfinite(state).all() and
                    (np.abs(state[2:]) < 100).all() and
                    (height > -0.3) and # height starts at 0 in pybullet
                    (abs(ang) < .2))

        debugmode = 0
        if debugmode:
            print("potential=")
            print(potential)
            print("power_cost=")
            print(power_cost)

        self.rewards = [
            potential,
            alive_bonus,
            power_cost
        ]
        if debugmode:
            print("rewards=")
            print(self.rewards)
            print("sum rewards")
            print(sum(self.rewards))
        self.HUD(state, a, done)
        self.reward += sum(self.rewards)

        return state, sum(self.rewards), bool(done), {}
