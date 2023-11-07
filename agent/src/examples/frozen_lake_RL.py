import os
import time

import gym
import matplotlib.pyplot as plt
import numpy as np
import QLearner as ql
from gym.envs.toy_text.frozen_lake import generate_random_map


def try_environment():
    for step in range(15):
        print(env.render())
        action = env.action_space.sample()
        observation, reward, done, trunc, info = env.step(action)
        time.sleep(0.2)
        os.system("cls")
        if done:
            env.reset()

    env.close()


def train(env):
    debug = False

    rewards = []
    log_interval = 1000

    learner = ql.QLearner(
        states=env.observation_space.n, actions=env.action_space.n, radr=0.001
    )

    for episode in range(EPOCHS):
        if debug:
            print(
                f"============= running episode: {episode} of {EPOCHS} ================="
            )

        state = env.reset()[0]
        done = False
        total_rewards = 0
        action = learner.get_next_action_without_Q_table_update(state)

        while not done:
            # state, reward... env.step()
            new_state, reward, done, trunc, info = env.step(action)

            # get next action
            action = learner.get_next_action_with_Q_table_update(new_state, reward)

            # track rewards
            total_rewards += reward

        # if debug: print(learner.Q)

        # agent finished a round of the game
        episode += 1

        # decay the random action rate
        learner.decay_rar(episode)

        rewards.append(total_rewards)

        if episode % log_interval == 0:
            print(np.sum(rewards))

    env.close()
    return learner.Q


# lets see how the agent is performing after training
def check_performance(env, use_q_table=False, q_table=None):
    state = env.reset()[0]

    for steps in range(100):
        print(env.render())
        if use_q_table == True:
            action = np.argmax(q_table[state, :])
        else:
            action = env.action_space.sample()
        state, reward, done, trunc, info = env.step(action)
        time.sleep(0.5)
        os.system("cls")

        if done:
            break

    env.close()


if __name__ == "__main__":
    # try_environment()

    # create a random environment
    env = gym.make(
        "FrozenLake-v1",
        desc=generate_random_map(size=5),
        is_slippery=False,
        render_mode="ansi",
        max_episode_steps=1000,
    )  # max actions to take before stopping the game

    # create a default environment
    # env = gym.make('FrozenLake-v1', desc=None, map_name='8x8', is_slippery=False, render_mode='ansi')

    training = True
    testing = True

    if training == True:
        # env.reset()
        EPOCHS = 10000  # episodes, how many times the agents plays the game until it hits done
        q_table = train(env)
        np.save("C:\\Users\\riteshm\omscs\\vip\\RL\\frozen_lake_q_table.npy", q_table)

    if testing == True:
        # print("====== using random actions... ")
        # time.sleep(2)
        # env.reset()
        # check_performance(env, use_q_table=False)

        time.sleep(2)
        print("====== using Q-table actions... ")
        time.sleep(2)
        # env.reset()
        q_table = np.load("C:\\Users\\riteshm\omscs\\vip\\RL\\frozen_lake_q_table.npy")
        check_performance(env, use_q_table=True, q_table=q_table)
