from stable_baselines3.common.env_checker import check_env
from morse_simulator.gym_wrapper.robot_morse_env import RobotMorseEnv
from morse_simulator.algorithms.config import config

env = RobotMorseEnv()
# It will check your custom environment and output additional warnings if needed
check_env(env)