import math
from gym import spaces
import numpy as np

class Env:
    def __init__(self):
        high = np.array([300, 300])
        low = - high

        self.action_space = spaces.Box(low=low, high=high, dtype=np.float32)
        self.observation_space = spaces.Box(low, high, dtype=np.float32)
        self.goal = [-0.3, -0.09]
        self.obs1 = [-0.0875, 0.1375]
        self.obs2 = [-0.2345, 0.02933]
        self.obs3 = [-0.137, -0.0987]

    def calc_dist(self, target, state):
        distx = target[0] - state[0][0]
        disty = target[1] - state[0][1]
        dist = math.sqrt(distx ** 2 + disty ** 2)
        return dist

    def calc_shaped_reward(self, state):
        reward = 0
        done = False
        obs_hit = False
        self.goal_reward = 500

        goaldist = self.calc_dist(self.goal, state)
        obs1dist = self.calc_dist(self.obs1, state)
        obs2dist = self.calc_dist(self.obs2, state)
        obs3dist = self.calc_dist(self.obs3, state)


        # if state[0][1] > 0.2 or state[0][1] < -0.24:
        #     reward += -10
        #     #done = True
        # elif state[0][0] > 0.075 or state[0][0] < -0.35:
        #     reward += -10
        #     #done = True
        # else:

        if obs1dist <= 0.0375 or obs2dist <= 0.0375 or obs3dist <= 0.0375:
            reward += -1

        if goaldist < 0.02:
            reward += self.goal_reward
            print("--- Goal reached!! ---")
            done = True
        else:
            reward += -goaldist




        # if math.sqtr(np.mean(buffer.memory[-30:].reward.numpy())**2 - reward**2) < 0.03:
        #     for i in range(1,32):
        #        buffer.memory[-i].reward = -20


        return reward, done, obs_hit

    def calc_non_shaped_reward(self, state):
        reward = 0
        done = False
        distx = self.goal[0] - state[0][0]
        disty = self.goal[1] - state[0][1]
        dist = math.sqrt(distx ** 2 + disty ** 2)

        if state[0][1] > 0.2 or state[0][1] < -0.24:
            reward += -10
            #done = True
        elif state[0][0] > 0.075 or state[0][0] < -0.35:
            reward += -10
            #done = True
        else:
            if dist < 0.02:
                reward += self.goal_reward
                print("--- Goal reached!! ---")
                done = True
            else:
                reward += -0.1

        return reward, done

