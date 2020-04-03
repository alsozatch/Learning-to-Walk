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
plane_urdf = "simpleplane.urdf"

class TestGymEnv(gym.Env):
    metadata = {'render.modes': ['human', 'rgb_array'], 'video_frames_per_second' : 50}
 
    def __init__(self):
        self._observation = []
        self.action_space = Tuple([spaces.Discrete(9), spaces.Discrete(9)])
        # pitch, gyro, commanded speed
        self.observation_space = spaces.Box(np.array([-math.pi, -math.pi, -5]), np.array([math.pi, math.pi, 5]))
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF
        self._seed()
    
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
       self.vd = 0
       self._envStepCounter = 0
    
       p.resetSimulation()
       p.setGravity(0,0,-9.81) # m/s^2
       p.setTimeStep(0.01) # sec
    
       planeId = p.loadURDF(plane_urdf)
       cubeStartPos = [0,0,0.001]
       cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
    
       path = os.path.abspath(os.path.dirname(__file__))
       self.botId = p.loadURDF(os.path.join(path, urdfname), cubeStartPos, cubeStartOrientation)
    
       self._observation = self._compute_observation()
       return np.array(self._observation)
 
    # used for visualization. empty because pybullet takes care of visualization if connected with the GUI flag
    def render(self, mode='human', close=False):
        pass

    # vt is velocity, vd is velocity desired, dv is a small change in velocity like in calculus
    def _assign_throttle(self, action):
        dv = 0.1
        deltav = [-10.*dv,-5.*dv, -2.*dv, -0.1*dv, 0, 0.1*dv, 2.*dv,5.*dv, 10.*dv]
        deltav0 = deltav[action.__getitem__(0)]
        deltav1 = deltav[action.__getitem__(1)]
        new_vt0 = self.vt0 + deltav0
        new_vt1 = self.vt1 + deltav1
        self.vt0 = new_vt0
        self.vt1 = new_vt1
        p.setJointMotorControl2(bodyUniqueId=self.botId, jointIndex=0, controlMode=p.VELOCITY_CONTROL, targetVelocity=new_vt0)
        p.setJointMotorControl2(bodyUniqueId=self.botId, jointIndex=1, controlMode=p.VELOCITY_CONTROL, targetVelocity=-new_vt1)

    def _compute_observation(self):
        cubePos, cubeOrn = p.getBasePositionAndOrientation(self.botId)
        cubeEuler = p.getEulerFromQuaternion(cubeOrn)
        linear, angular = p.getBaseVelocity(self.botId)
        return [cubeEuler[0],angular[0],self.vt0]
        
    def _compute_reward(self):
        _, cubeOrn = p.getBasePositionAndOrientation(self.botId)
        cubeEuler = p.getEulerFromQuaternion(cubeOrn)
        # could also be pi/2 - abs(cubeEuler[0])
        return (1 - abs(cubeEuler[0])) * 0.1 -  abs(self.vt0 - self.vd) * 0.01

    def _compute_done(self):
        cubePos, _ = p.getBasePositionAndOrientation(self.botId)
        return cubePos[2] < 0.15 or self._envStepCounter >= 1500
