from pybulletgym.envs.mujoco.envs.env_bases import BaseBulletEnv
from pybulletgym.envs.mujoco.robots.pendula.inverted_double_pendulum import InvertedDoublePendulum
from pybulletgym.envs.mujoco.scenes.scene_bases import SingleRobotEmptyScene


class InvertedDoublePendulumMuJoCoEnv(BaseBulletEnv):
    def __init__(self):
        self.robot = InvertedDoublePendulum()
        BaseBulletEnv.__init__(self, self.robot)
        self.stateId = -1

    def create_single_player_scene(self, bullet_client):
        return SingleRobotEmptyScene(bullet_client, gravity=9.8, timestep=0.0165, frame_skip=1)

    def reset(self):
        if self.stateId >= 0:
            self._p.restoreState(self.stateId)
        r = BaseBulletEnv._reset(self)
        if self.stateId < 0:
            self.stateId = self._p.saveState()
        return r

    def step(self, a):
        self.robot.apply_action(a)
        self.scene.global_step()
        state = self.robot.calc_state()
        # upright position: 0.6 (one pole) + 0.6 (second pole) * 0.5 (middle of second pole) = 0.9
        # using <site> tag in original xml, upright position is 0.6 + 0.6 = 1.2, difference +0.3
        pos_x, _, pos_y = self.robot.pole2.pose().xyz()
        dist_penalty = 0.01 * pos_x ** 2 + (pos_y + 0.3 - 2) ** 2
        v1, v2 = self.robot.j1.current_position()[1], self.robot.j2.current_position()[1]
        vel_penalty = 1e-3 * v1**2 + 5e-3 * v2**2
        alive_bonus = 10
        done = pos_y + 0.3 <= 1
        self.rewards = [float(alive_bonus), float(-dist_penalty), float(-vel_penalty)]
        self.HUD(state, a, done)
        return state, sum(self.rewards), done, {}

    def camera_adjust(self):
        self.camera.move_and_look_at(0, 1.2, 1.2, 0, 0, 0.5)
