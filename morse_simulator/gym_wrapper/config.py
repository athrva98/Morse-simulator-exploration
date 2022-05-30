# Dynamic Global variables.

robot_number = None
server_port = None
motion_port = None
pose_port = None
depthcamera_port = None
child_processes = None # this saves the PIDs of the child processes. These processes are killed when env.hardReset is called
occupancy_grid = None # this is the running occupancy grid during the simualtions.
initialPosition = None # this is the spawn position of the robots in the environment. (Used during env.softReset)
startTime = None # this is used to see if the episode has ended. We use a limited time episode.

""" Transactional items : This file is also used to share variables between different files of this workspace"""
status_dict = {} # holds the reward and the episode number.
episode_reward = None # redundant.