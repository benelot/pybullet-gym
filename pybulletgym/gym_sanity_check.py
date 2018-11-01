import gym
import numpy as np
import traceback
import .envs as pb_env

envs = [spec.id for spec in gym.envs.registry.all() if spec.id.find('PyBullet') >= 0]
bugged_envs = []
for env_name in envs:
    try:
        print('[TESTING] ENV', env_name, '...')
        env = gym.make(env_name)
        env.reset()
        env.step(np.random.random(env.action_space.shape))
        print('[SUCCESS] ENV', env_name, '\n')
    except Exception as e:
        print(env_name, ': ', traceback.format_exc())
        bugged_envs.append(env_name)
        print('[FAIL] ENV', env_name, '\n')

print('The following envs have problems:', bugged_envs)
