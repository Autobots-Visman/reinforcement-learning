import gymnasium as gym
from gymnasium import error, spaces, utils
from gymnasium.utils import seeding
import numpy as np

class HandyEnv(gym.Env):
    def __init__(self):
        super(HandyEnv, self).__init__()

        # cartesian displacement dx, dy, dz of the end effector in meters
        # 1. define the range of values each entry in the action can take. In this case, each action value can be any floating point number between low=-1 and low=1
        #    So, we can only move the end effector in the x, y, z directions by a maximum of 1 meter in each direction.
        # 2. define the shape of the action space. In this case, the action space is a 3-dimensional vector.
        self.action_space = spaces.Box(low=-1, high=1, shape=(3,), dtype="float32")

        # observation space is the current state of the environment, and should be a 3-dimensional vector representing the cartesian coordinates of the end effector in meters
        # with minimum value of 0 and maximum value of 5 meters in each direction
        self.observation_space = spaces.Box(low=0, high=5, shape=(3,))
        self._agent_location = None
        self._target_location = None
        self.time_limit = 1000
        self._iteration = 0

    def _get_obs(self):
        '''return the current state of the environment as an observation to the agent'''

        # some meaningful conversion of the state to an observation
        pass

    def _get_info(self):
        '''return some info about the environment, like the distance between the agent and the target, etc.'''
        return {}

    # distance between the agent and the target
    def _get_distance(self):
        """Calculate the euclidean distance between the agent and the target"""
        return np.linalg.norm(np.array(self._agent_location) - np.array(self._target_location))

    def reset(self):
        '''reset the environment to its initial state and return the initial observation and info to the agent'''

        # set agent to some pre-defined initial state/location, like the origin
        self._agent_location = np.array([1., 1., 1.], dtype=np.float32)

        # set target location the agent needs to reach to
        self._target_location = np.array([2., 2., 2.], dtype=np.float32) # some target location TODO: this should come from call to the API

        observation = self._agent_location
        info = self._get_info()

        return observation, info

    def step(self, action):
        '''take an action and return the new state, reward, done, and info to the agent'''

        assert self.action_space.contains(action), f"Action {action} is invalid"

        # TODO: send action to the api and get the new state
        # for now randomly update provide an observation by randomly sampling from the allowed observation space
        self._agent_location = np.array([np.random.uniform(0, 5), np.random.uniform(0, 5), np.random.uniform(0, 5)], dtype=np.float32)

        # should set some time limit for the robot to reach the target
        self._iteration += 1
        self.time_limit -= 1
        if self.time_limit == 0:
            print("Time limit reached")
            done = True
        else:
            done = self._get_distance() < 0.1   # some threshold
            print(f"Step: {self._iteration}, Agent location: {self._agent_location}, Target location: {self._target_location}, Distance: {self._get_distance()}")

        # TODO: need to think on how to get the reward
        reward = 1.0 if done else 0.0

        observation = self._agent_location
        info = self._get_info()

        return observation, reward, done, False, info

    def render(self, mode='human'):
        '''render the environment to the screen or to a file, etc.'''
        pass

    def close(self):
        '''close the environment and release resources if any'''
        pass