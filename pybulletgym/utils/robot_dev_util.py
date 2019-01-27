from pybulletgym.envs.roboschool.robots.locomotors import Ant
import gym.utils.seeding
import numpy as np
import pybullet as p
import time
from importlib import import_module
import sys


if __name__ == "__main__":

    physicsClient = p.connect(p.GUI)

    path, class_str = sys.argv[1].rsplit('.', 1)
    module = import_module(path)
    robot_class = getattr(module, class_str)

    robot = robot_class()
    np_random, seed = gym.utils.seeding.np_random()
    robot.np_random = np_random
    robot.reset(p)

    while True:
        try:
            robot.apply_action(np.random.uniform(robot.action_space.low, robot.action_space.high, robot.action_space.shape))
            p.stepSimulation()
            time.sleep(1./240.)
        except KeyboardInterrupt:
            p.disconnect()
