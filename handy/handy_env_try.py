import gymnasium as gym
import handy

env = gym.make('Handy-v0')
observation, _ = env.reset()
print(observation)

while True:
    action = env.action_space.sample()
    observation, reward, done, _, _ = env.step(action)
    if done:
        break

