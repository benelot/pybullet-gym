import gym
import pybulletgym  # required to register the pybullet envs
import traceback
from pybulletgym.tests.roboschool.agents.policies import SmallReactivePolicy

import pybulletgym.tests.roboschool.agents.AntPyBulletEnv_v0_2017may as AntWeights
import pybulletgym.tests.roboschool.agents.AtlasPyBulletEnv_v0_2017jul as AtlasWeights
import pybulletgym.tests.roboschool.agents.HalfCheetahPyBulletEnv_v0_2017may as HalfCheetahWeights
import pybulletgym.tests.roboschool.agents.HopperPyBulletEnv_v0_2017may as HopperWeights
import pybulletgym.tests.roboschool.agents.HumanoidPyBulletEnv_v0_2017may as HumanoidWeights
import pybulletgym.tests.roboschool.agents.HumanoidFlagrunPyBulletEnv_v0_2017may as HumanoidFlagrunWeights
import pybulletgym.tests.roboschool.agents.HumanoidFlagrunHarderPyBulletEnv_v1_2017jul as HumanoidFlagrunHarderWeights
import pybulletgym.tests.roboschool.agents.InvertedDoublePendulumPyBulletEnv_v0_2017may as InvertedDoublePendulumWeights
import pybulletgym.tests.roboschool.agents.InvertedPendulumPyBulletEnv_v0_2017may as InvertedPendulumWeights
import pybulletgym.tests.roboschool.agents.InvertedPendulumSwingupPyBulletEnv_v0_2017may as InvertedPendulumSwingupWeights
import pybulletgym.tests.roboschool.agents.ReacherPyBulletEnv_v0_017may as ReacherWeights
import pybulletgym.tests.roboschool.agents.Walker2DPyBulletEnv_v0_2017may as WalkerWeights


envs = [spec.id for spec in gym.envs.registry.all() if spec.id.find('Bullet') >= 0]

env_performance = {
    # pendula
    'InvertedPendulumPyBulletEnv-v0': 999,
    'InvertedDoublePendulumPyBulletEnv-v0': 9344,
    'InvertedPendulumSwingupPyBulletEnv-v0': 836,

    # manipulators
    'ReacherPyBulletEnv-v0': 18,
    'PusherPyBulletEnv-v0': 10,
    'ThrowerPyBulletEnv-v0': 10,
    'StrikerPyBulletEnv-v0': 10,

    # locomotors
    'Walker2DPyBulletEnv-v0': 19,
    'HalfCheetahPyBulletEnv-v0': 10,
    'AntPyBulletEnv-v0': 20,
    'HopperPyBulletEnv-v0': 423,
    'HumanoidPyBulletEnv-v0': 41,
    'HumanoidFlagrunPyBulletEnv-v0': 86,
    'HumanoidFlagrunHarderPyBulletEnv-v0': 10,
    'AtlasPyBulletEnv-v0': 10,
}

weights = {
    # pendula
    'InvertedPendulumPyBulletEnv-v0': [[InvertedPendulumWeights.weights_dense1_w,
                                        InvertedPendulumWeights.weights_dense2_w,
                                        InvertedPendulumWeights.weights_final_w],
                                       [InvertedPendulumWeights.weights_dense1_b,
                                        InvertedPendulumWeights.weights_dense2_b,
                                        InvertedPendulumWeights.weights_final_b]],
    'InvertedDoublePendulumPyBulletEnv-v0': [[InvertedDoublePendulumWeights.weights_dense1_w,
                                              InvertedDoublePendulumWeights.weights_dense2_w,
                                              InvertedDoublePendulumWeights.weights_final_w],
                                             [InvertedDoublePendulumWeights.weights_dense1_b,
                                              InvertedDoublePendulumWeights.weights_dense2_b,
                                              InvertedDoublePendulumWeights.weights_final_b]],
    'InvertedPendulumSwingupPyBulletEnv-v0': [[InvertedPendulumSwingupWeights.weights_dense1_w,
                                               InvertedPendulumSwingupWeights.weights_dense2_w,
                                               InvertedPendulumSwingupWeights.weights_final_w],
                                              [InvertedPendulumSwingupWeights.weights_dense1_b,
                                               InvertedPendulumSwingupWeights.weights_dense2_b,
                                               InvertedPendulumSwingupWeights.weights_final_b]],

    # manipulators
    'ReacherPyBulletEnv-v0': [[ReacherWeights.weights_dense1_w,
                               ReacherWeights.weights_dense2_w,
                               ReacherWeights.weights_final_w],
                              [ReacherWeights.weights_dense1_b,
                               ReacherWeights.weights_dense2_b,
                               ReacherWeights.weights_final_b]],
    # 'PusherPyBulletEnv-v0': [[InvertedPendulumWeights.weights_dense1_w,
    #                           InvertedPendulumWeights.weights_dense2_w,
    #                           InvertedPendulumWeights.weights_final_w],
    #                          [InvertedPendulumWeights.weights_dense1_b,
    #                           InvertedPendulumWeights.weights_dense2_b,
    #                           InvertedPendulumWeights.weights_final_b]],
    # 'ThrowerPyBulletEnv-v0': [[InvertedPendulumWeights.weights_dense1_w,
    #                            InvertedPendulumWeights.weights_dense2_w,
    #                            InvertedPendulumWeights.weights_final_w],
    #                           [InvertedPendulumWeights.weights_dense1_b,
    #                            InvertedPendulumWeights.weights_dense2_b,
    #                            InvertedPendulumWeights.weights_final_b]],
    # 'StrikerPyBulletEnv-v0': [[InvertedPendulumWeights.weights_dense1_w,
    #                            InvertedPendulumWeights.weights_dense2_w,
    #                            InvertedPendulumWeights.weights_final_w],
    #                           [InvertedPendulumWeights.weights_dense1_b,
    #                            InvertedPendulumWeights.weights_dense2_b,
    #                            InvertedPendulumWeights.weights_final_b]],

    # locomotors
    'Walker2DPyBulletEnv-v0': [[WalkerWeights.weights_dense1_w,
                                WalkerWeights.weights_dense2_w,
                                WalkerWeights.weights_final_w],
                               [WalkerWeights.weights_dense1_b,
                                WalkerWeights.weights_dense2_b,
                                WalkerWeights.weights_final_b]],
    'HalfCheetahPyBulletEnv-v0': [[HalfCheetahWeights.weights_dense1_w,
                                   HalfCheetahWeights.weights_dense2_w,
                                   HalfCheetahWeights.weights_final_w],
                                  [HalfCheetahWeights.weights_dense1_b,
                                   HalfCheetahWeights.weights_dense2_b,
                                   HalfCheetahWeights.weights_final_b]],
    'AntPyBulletEnv-v0': [[AntWeights.weights_dense1_w,
                           AntWeights.weights_dense2_w,
                           AntWeights.weights_final_w],
                          [AntWeights.weights_dense1_b,
                           AntWeights.weights_dense2_b,
                           AntWeights.weights_final_b]],
    'HopperPyBulletEnv-v0': [[HopperWeights.weights_dense1_w,
                              HopperWeights.weights_dense2_w,
                              HopperWeights.weights_final_w],
                             [HopperWeights.weights_dense1_b,
                              HopperWeights.weights_dense2_b,
                              HopperWeights.weights_final_b]],
    'HumanoidPyBulletEnv-v0': [[HumanoidWeights.weights_dense1_w,
                                HumanoidWeights.weights_dense2_w,
                                HumanoidWeights.weights_final_w],
                               [HumanoidWeights.weights_dense1_b,
                                HumanoidWeights.weights_dense2_b,
                                HumanoidWeights.weights_final_b]],
    'HumanoidFlagrunPyBulletEnv-v0': [[HumanoidFlagrunWeights.weights_dense1_w,
                                       HumanoidFlagrunWeights.weights_dense2_w,
                                       HumanoidFlagrunWeights.weights_final_w],
                                      [HumanoidFlagrunWeights.weights_dense1_b,
                                       HumanoidFlagrunWeights.weights_dense2_b,
                                       HumanoidFlagrunWeights.weights_final_b]],
    'HumanoidFlagrunHarderPyBulletEnv-v0': [[HumanoidFlagrunHarderWeights.weights_dense1_w,
                                             HumanoidFlagrunHarderWeights.weights_dense2_w,
                                             HumanoidFlagrunHarderWeights.weights_final_w],
                                            [HumanoidFlagrunHarderWeights.weights_dense1_b,
                                             HumanoidFlagrunHarderWeights.weights_dense2_b,
                                             HumanoidFlagrunHarderWeights.weights_final_b]],
    'AtlasPyBulletEnv-v0': [[AtlasWeights.weights_dense1_w,
                             AtlasWeights.weights_dense2_w,
                             AtlasWeights.weights_final_w],
                            [AtlasWeights.weights_dense1_b,
                             AtlasWeights.weights_dense2_b,
                             AtlasWeights.weights_final_b]],
}

test_steps = 5000

underachieving_envs = []
bugged_envs = []
for env_name in envs:
    try:
        print('[TESTING] ENV', env_name, '...')

        # setup env
        env = gym.make(env_name)
        env.seed(7)   # Fix random seed to achieve determinism
        obs = env.reset()

        # setup agent
        agent = SmallReactivePolicy(env.observation_space,
                                    env.action_space,
                                    weights[env_name][0],  # weights
                                    weights[env_name][1])  # biases

        total_reward = 0
        for i in range(0, test_steps):
            a = agent.act(obs)
            obs, r, done, _ = env.step(a)

            if done:
                break
            total_reward += r

        if total_reward >= env_performance[env_name]:
            print('[SUCCESS] ENV')
        else:
            print('[FAIL] ENV')
            underachieving_envs.append(env_name)

        # print some statistics
        print(env_name, '/ total reward: ',
                  total_reward, '/', env_performance[env_name],
                  '(', total_reward / env_performance[env_name] * 100, '%)', '\n')

    except Exception as e:
        print(env_name, ': ', traceback.format_exc())
        bugged_envs.append(env_name)
        print('[FAIL] ENV', env_name, '\n')

print('The following envs perform worse:', underachieving_envs, '\n')
print('The following envs have problems:', bugged_envs)
