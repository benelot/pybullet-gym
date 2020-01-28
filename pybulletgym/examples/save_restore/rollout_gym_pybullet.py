""" An example of a Monte-Carlo rollout from a given state envolving saving
    and loading the state of the environment...
"""
import time
import numpy as np
import torch
import torch.multiprocessing as mp
import gym
import pybulletgym
np.set_printoptions(precision=4, suppress=True)


MAX_STEPS = 1000


def mc_rollout(env_name, state_path, crt_step):
    np.set_printoptions(precision=4, suppress=True)
    env = gym.make(env_name)
    obs = env.reset()

    env.restore_env_state(state_path)
    print(f"[rollout] Loaded state from {state_path}")

    Gt, step, done, first_action = 0, 0, False, None
    while not done:
        action = env.action_space.sample()
        obs, reward, done, _ = env.step(action)

        if step % 100 == 0:
            print(f"[rollout]: Did {step} steps in the env.")
        
        if step == 0:
            print(f"\n[rollout]: First observation OF the rollout:\n", obs, "\n")
            first_action = action

        Gt += reward
        step += 1
        if step == (MAX_STEPS - crt_step):
            break

    env.close()

    print(f"\n[rollout] done after {step} steps, return={Gt:3.2f}.")
    return Gt, step, first_action


def main():
    pool = mp.Pool(processes=1)

    # env_name = "AntPyBulletEnv-v0"
    # env_name = "AntMuJoCoEnv-v0"
    # env_name = "ReacherPyBulletEnv-v0"
    env_name = "HalfCheetahMuJoCoEnv-v0"

    env = gym.make(env_name)

    # env.render(mode="human")
    obs, done, roll_act = env.reset(), False, None
    Gt, step = 0, 0
    while not done:
        if step % 100 == 0:
            print(f"[main] Did {step} steps.")

        action = env.action_space.sample() if roll_act is None else roll_act
        obs, reward, done, _ = env.step(action)

        if step == 50:
            state_path = f"/run/shm/state_{step}.bullet"
            env.save_env_state(state_path)
            # start a monte-carlo rollout
            task = pool.starmap_async(mc_rollout, [(env_name, state_path, step)])
            # and wait for it to finish
            Gmc, Hmc, roll_act = task.get()[0]
            print("[main] Rollout returned: ", Gmc, Hmc)
        
        if step == 51:
            print(f"\n[main] First observation AFTER rollout:\n", obs, "\n")

        if step == MAX_STEPS:
            break

        Gt += reward
        step += 1

    print(f"Done after {step} steps, return={Gt:3.2f}.")


if __name__ == "__main__":
    main()
