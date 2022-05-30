import gym
import numpy as np
from gym.utils import seeding
from gym import spaces
from morse_simulator.gym_wrapper.morse_environment import morseConnection
from morse_simulator.gym_wrapper.controllers import controllerInterface
from morse_simulator.algorithms.costmap import Costmap
from morse_simulator.gym_wrapper.controllers import controllerInterface
import time
from morse_simulator.algorithms.config import config
import morse_simulator.gym_wrapper.config as dynamic_cfg
from morse_simulator.gym_wrapper import DepthCamera
from pymorse import Morse # this allows us to add functionality like pausing the simulation, reseting objects etc.


class RobotMorseEnv(gym.Env):
    
    def __init__(self):
        super(RobotMorseEnv, self).__init__()
        self.morse = morseConnection()
        self.controllerInterface = controllerInterface()
        self._generate_action_space()
        self.map = DepthCamera.map_t()
        
        self.dcam = DepthCamera.DepthCameraPose(robot_number = 0, pose_port = dynamic_cfg.pose_port,
                                                depth_camera_port = dynamic_cfg.depthcamera_port)
        
        self.observation_space = spaces.Box(low = 0., high = 1.,
                                            shape = self.map.cells.shape, dtype = np.float16)
        
        self.reward_msg = {} # holds the reward information
        self.episode_num = 0 # episode tracker
        self.cumulated_episode_reward = 0
    
    def _generate_action_space(self):
        if config.controller == 'teleport':
            # TODO : set thresholds in the config.
            low = np.array([-100., -100., 0, 0., -np.pi/5, -np.pi/5]).astype(np.float16)
            high = np.array([100., 100., 11, 2*np.pi, np.pi/5, np.pi/5]).astype(np.float16)
            self.action_space = spaces.Box(low = low, high = high)
        if config.controller == 'waypoint':
            low = np.array([-100., -100., 0., 0.]).astype(np.float16)
            high = np.array([100., 100., 11, 2*np.pi]).astype(np.float16)
            self.action_space = spaces.Box(low = low, high = high)
        if config.controller == 'engine-speed':
            low = np.array([0., 0., 0., 0.]).astype(np.float16)
            high = np.array([100., 100., 100., 100.]).astype(np.float16)
            self.action_space = spaces.Box(low = low, high = high)
        if config.controller == 'attitude':
            low = np.array([-np.pi/5, -np.pi/5, 0., 0.]).astype(np.float16)
            high = np.array([np.pi/5, np.pi/5, 2*np.pi, 100.]).astype(np.float16)
            self.action_space = spaces.Box(low = low, high = high)
        if config.controller == 'velocity':
            low = np.array([0., 0., 0., 0.]).astype(np.float16)
            high = np.array([100., 100., 100., 100.]).astype(np.float16)
            self.action_space = spaces.Box(low = low, high = high)
    
    
    
    def step(self, action):
        
        self._set_action(action)
        
        obs = self._get_obs()
        done = self._is_done()
        info = {}
        reward = self._compute_reward(obs, done)
        self.cumulated_episode_reward += reward
        info['observation'] = obs
        info['done'] = done
        info['reward'] = reward
        return obs, reward, done, info
    
    def _get_obs(self):
        self.dcam.points_cloud_update()
        _, _, _ = self.dcam.occupancy_grid_update()
        return dynamic_cfg.occupancy_grid
    
    def render(self, mode='human', close = False):
        print('=.=.=.=.=.=.=.=.=.=.=.=.=.=.=.=.=.=.=.=.=.=')
        print('Step : \t', self.episode_num)
        print('Reward : \t', self.cumulated_episode_reward)
        print('\n')
        return
        
    def reset(self):
        self._reset_sim()
        self._update_episode()
        
        obs = self._get_obs()
        
        return obs
    
    def close(self):
        # TODO implement this in a process safe manner.
        dynamic_cfg.close(dynamic_cfg.child_processes) # kills all the child processes.
    
    
    def _update_episode(self):
        self._publish_reward_topic(self.cumulated_episode_reward,
                                  self.episode_num)
        dynamic_cfg.episode_reward = self.cumulated_episode_reward
        self.episode_num += 1
        self.cumulated_episode_reward = 0
        
    
    def _publish_reward_topic(self, reward, episode_number = 1):
        self.reward_msg['reward'] = reward
        self.reward_msg['episode_number'] = episode_number
        dyncmic_cfg.status_dict = self.reward_msg
        
    def _reset_sim(self, rtype = 'soft'):
        if rtype == 'soft':
            self.morse.softResetSim()
        if rtype == 'hard':
            self.morse.hardResetSim()
        return True
    
    def _isValidAction(self):
        # Is the furnished action valid, given the controller?
        
        if self.controller == 'engine-speed':
            """ This controller expects angular velocity in rad/s"""
            assert len(action) == 4
        if self.controller == 'attitude':
            """ This controller expects roll, pitch, yaw, thrust"""
            assert len(action) == 4
        if self.controller ==  'velocity':
            """ This controller expects vx, vy, vz, vyaw, tolerance"""
            assert len(action) == 4
        if self.controller == 'waypoint':
            """ This controller expects x, y, z, yaw """
            assert len(action) == 4
        if self.controller == 'teleport':
            """ This controller expects a certain translation and rotation"""
            # It is not recommended to use this controller as it does not
            # implement the quadrotor dynamics.
            assert len(action) == 6
        return True # if you reached this far, the action must be valid
    
    def _set_action(self, action):
        """
        action is basically a command based on the controller.
        """
        self._isValidAction(action)
        self.controllerInterface.set_action(action) # TODO : Implement this control interface for all the controllers.
    
    
    def checkSabotage(self): # this is a tricky function.
        # We check if the robot has become unstable and soft reset if such is the case
        # this counts as a fail and a neative reward is associated with this.
        status = self.morse.checkSabotage()
        return status
        
        
        
    def _is_done(self):
        # TODO : Some way to furnish the time of simulation.
        if time.time() - config.startTime >= config.maxTime:
            self.close()
            return True
        if self.checkSabotage(): # if the robot became unstable and the simulation quit.
            self.morse.softResetSim() # do a soft reset.
            return True
        return False
    def _compute_reward(self, obs):
        # TODO : implement this
        # Basically just the area explored at the end of each episode
        reward = len(np.where(obs == Costmap.UNOCCUPIED)[0])
        return reward
    
        


        
    
        
        
        
        
