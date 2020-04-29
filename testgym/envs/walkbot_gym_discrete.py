import gym
from gym import error, spaces, utils
from gym.spaces import Discrete, Tuple
from gym.utils import seeding

import os
import math
import numpy as np
import pybullet as p
import pybullet_data

robot_urdf = "walkbot.urdf"
plane_urdf = "plane.urdf"

class TestGymEnv(gym.Env):
    metadata = {'render.modes': ['human', 'rgb_array'], 'video_frames_per_second' : 50}
 
    def __init__(self):
        self._observation = []
        self.action_space = spaces.Discrete(81)
        # pitch, gyro, commanded speed
        self.observation_space = spaces.Box(np.array([-math.pi, -math.pi, -5]), np.array([math.pi, math.pi, 5]))
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF
        self._seed()
        self._envEpisodeCounter = 0
    
    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
    
    def _step(self, action):
        self._assign_throttle(action)
        p.stepSimulation()
        self._observation = self._compute_observation()
        reward = self._compute_reward()
        done = self._compute_done()
  
        self._envStepCounter += 1
  
        return np.array(self._observation), reward, done, {}
 
    def _reset(self):
        self.vt0 = 0
        self.vt1 = 0
        self.vt2 = 0
        self.vt3 = 0
        self.vd = 0
        self._envStepCounter = 0
    
        p.resetSimulation()
        p.setGravity(0,0,-9.81) # m/s^2
        p.setTimeStep(0.01) # sec
    
        planeId = p.loadURDF(plane_urdf)
        cubeStartPos = [0,0,0.001]
        cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
    
        path = os.path.abspath(os.path.dirname(__file__))
        self.botId = p.loadURDF(os.path.join(path, robot_urdf), cubeStartPos, cubeStartOrientation)
    
        self._observation = self._compute_observation()
       
        self._envEpisodeCounter += 1
        print(self._envEpisodeCounter)
        return np.array(self._observation)
 
    # used for visualization. empty because pybullet takes care of visualization if connected with the GUI flag
    def render(self, mode='human', close=False):
        pass

    # vt is velocity, vd is velocity desired, dv is a small change in velocity like in calculus
    def _assign_throttle(self, action):
        options = [[-0.1,-0.1,-0.1,-0.1], [-0.1,-0.1,-0.1,0], [-0.1,-0.1,-0.1,0.1], [-0.1,-0.1,0,-0.1], [-0.1,-0.1,0,0], [-0.1,-0.1,0,0.1], [-0.1,-0.1,0.1,-0.1], [-0.1,-0.1,0.1,0], [-0.1,-0.1,0.1,0.1], [-0.1,0,-0.1,-0.1], [-0.1,0,-0.1,0], [-0.1,0,-0.1,0.1], [-0.1,0,0,-0.1], [-0.1,0,0,0], [-0.1,0,0,0.1], [-0.1,0,0.1,-0.1], [-0.1,0,0.1,0], [-0.1,0,0.1,0.1], [-0.1,0.1,-0.1,-0.1], [-0.1,0.1,-0.1,0], [-0.1,0.1,-0.1,0.1], [-0.1,0.1,0,-0.1], [-0.1,0.1,0,0], [-0.1,0.1,0,0.1], [-0.1,0.1,0.1,-0.1], [-0.1,0.1,0.1,0], [-0.1,0.1,0.1,0.1], [0,-0.1,-0.1,-0.1], [0,-0.1,-0.1,0], [0,-0.1,-0.1,0.1], [0,-0.1,0,-0.1], [0,-0.1,0,0], [0,-0.1,0,0.1], [0,-0.1,0.1,-0.1], [0,-0.1,0.1,0], [0,-0.1,0.1,0.1], [0,0,-0.1,-0.1], [0,0,-0.1,0], [0,0,-0.1,0.1], [0,0,0,-0.1], [0,0,0,0], [0,0,0,0.1], [0,0,0.1,-0.1], [0,0,0.1,0], [0,0,0.1,0.1], [0,0.1,-0.1,-0.1], [0,0.1,-0.1,0], [0,0.1,-0.1,0.1], [0,0.1,0,-0.1], [0,0.1,0,0], [0,0.1,0,0.1], [0,0.1,0.1,-0.1], [0,0.1,0.1,0], [0,0.1,0.1,0.1], [0.1,-0.1,-0.1,-0.1], [0.1,-0.1,-0.1,0], [0.1,-0.1,-0.1,0.1], [0.1,-0.1,0,-0.1], [0.1,-0.1,0,0], [0.1,-0.1,0,0.1], [0.1,-0.1,0.1,-0.1], [0.1,-0.1,0.1,0], [0.1,-0.1,0.1,0.1], [0.1,0,-0.1,-0.1], [0.1,0,-0.1,0], [0.1,0,-0.1,0.1], [0.1,0,0,-0.1], [0.1,0,0,0], [0.1,0,0,0.1], [0.1,0,0.1,-0.1], [0.1,0,0.1,0], [0.1,0,0.1,0.1], [0.1,0.1,-0.1,-0.1], [0.1,0.1,-0.1,0], [0.1,0.1,-0.1,0.1], [0.1,0.1,0,-0.1], [0.1,0.1,0,0], [0.1,0.1,0,0.1], [0.1,0.1,0.1,-0.1], [0.1,0.1,0.1,0], [0.1,0.1,0.1,0.1]]
        deltav = options[action]
        self.vt0 = self.vt0 + deltav[0]
        self.vt1 = self.vt1 + deltav[1]
        self.vt2 = self.vt2 + deltav[2]
        self.vt3 = self.vt3 + deltav[3]
        p.setJointMotorControl2(bodyUniqueId=self.botId, jointIndex=0, controlMode=p.VELOCITY_CONTROL, targetVelocity=self.vt0)
        p.setJointMotorControl2(bodyUniqueId=self.botId, jointIndex=1, controlMode=p.VELOCITY_CONTROL, targetVelocity=self.vt1)
        p.setJointMotorControl2(bodyUniqueId=self.botId, jointIndex=2, controlMode=p.VELOCITY_CONTROL, targetVelocity=self.vt2)
        p.setJointMotorControl2(bodyUniqueId=self.botId, jointIndex=3, controlMode=p.VELOCITY_CONTROL, targetVelocity=self.vt3)

    def _compute_observation(self):
        pos, orn = p.getBasePositionAndOrientation(self.botId)
        euler = p.getEulerFromQuaternion(orn)
        linear, angular = p.getBaseVelocity(self.botId)
        return [euler[0],angular[0],self.vt0]
        
    def _compute_reward(self):
        pos, orn = p.getBasePositionAndOrientation(self.botId)
        return pos[1]

    def _compute_done(self):
        pos, _ = p.getBasePositionAndOrientation(self.botId)
        return pos[2] < -0.25 or self._envStepCounter >= 3000
