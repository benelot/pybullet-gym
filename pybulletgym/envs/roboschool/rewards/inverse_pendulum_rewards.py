import numpy as np


def calc_reward(robot):

    state = robot.calc_state()  # sets self.pos_x self.pos_y
    if robot.swingUp:
        reward = np.cos(robot.theta)
        done = False
    else:
        reward = 1.0
        done = np.abs(robot.theta) > .2

    return reward, done
