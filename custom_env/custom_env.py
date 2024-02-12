import gymnasium as gym
from gym import spaces

class CustomEnv(gym.Env):
    def __init__(self):
        super(CustomEnv, self).__init__()
        self.action_space = spaces.Discrete(2)
        self.observation_space = spaces.Box(low=0, high=1, shape=(3,))      # optional
        self.state = None
        self._agent_location = None
        self._target_location = None
        self.time_limit = 1000

    def _get_obs(self):
        '''return the current state of the environment as an observation to the agent'''

        # some meaningful conversion of the state to an observation
        d = {"agent": self._agent_location, "target": self._target_location}

        return d

    def _get_info(self):
        '''return some info about the environment, like the distance between the agent and the target, etc.'''
        return None

    def reset(self):
        '''reset the environment to its initial state and return the initial observation and info to the agent'''

        # set agent to some pre-defined initial state/location
        self._agent_location = None

        # set target location the agent needs to reach to
        self._target_location = None

        observation = self._get_obs()
        info = self._get_info()

        return observation, info

    # distance between the agent and the target
    def _get_distance(self):
        return None

    def step(self, action):
        '''take an action and return the new state, reward, done, and info to the agent'''

        assert self.action_space.contains(action), "%r (%s) invalid" % (action, type(action))

        # send action to the api and get the new state
        self.state = None

        # should set some time limit for the robot to reach the target
        if self.time_limit == 0:
            done = True
        done = self._get_distance() < 0.1   # some threshold

        # need to think on how to get the reward
        reward = None

        observation = self._get_obs()
        info = self._get_info()

        return observation, reward, done, False, info

    def render(self, mode='human'):
        '''render the environment to the screen or to a file, etc.'''
        pass

    def close(self):
        '''close the environment and release resources if any'''
        pass