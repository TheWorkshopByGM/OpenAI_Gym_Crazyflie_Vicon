import gym
from gym import spaces
import numpy as np
from gym_drone.envs.crazyflie import Crazyflie_Sim

class CustomEnv(gym.Env):
    def __init__(self):
        self.crazyflie = Crazyflie_Sim()
        self.crazyflie = spaces.Discrete(8) #CHANGE THAT TO THE NUMBER OF DISCRETE ACTIONS YOUR AI CAN DO (In this example we created 8 actions, to increase or decrease the speed of each of the 4 motors)
        self.observation_space = spaces.Box(np.array([0, 0, 0, 0, 0]), np.array([10, 10, 10, 10, 10]), dtype=np.int) #CHANGE THAT TO THE NUMBER OF DISCRETE OBSERVATIONS YOUR AI CAN SEE

    def reset(self):
        del self.crazyflie
        self.crazyflie = Crazyflie_Sim()
        obs = self.crazyflie.observe()
        return obs

    def step(self, action):
        self.crazyflie.action(action)
        obs = self.crazyflie.observe()
        reward = self.crazyflie.evaluate()
        done = self.crazyflie.is_done()
        return obs, reward, done, {}

    def render(self, mode="human", close=False):
        self.crazyflie.view()