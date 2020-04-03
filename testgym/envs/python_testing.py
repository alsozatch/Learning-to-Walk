import gym
from gym import spaces
from gym.spaces import Tuple
action_space = Tuple(spaces.Discrete(9), spaces.Discrete(9))
#action = Tuple(0, 1)

dv = 0.1
action = 0
deltav = [-10.*dv,-5.*dv, -2.*dv, -0.1*dv, 0, 0.1*dv, 2.*dv,5.*dv, 10.*dv][action]
print(deltav)
