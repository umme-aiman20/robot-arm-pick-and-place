import pybullet as p
import pybullet_data
import time
import numpy as np
from stable_baselines3 import PPO
from env.robot_env import RobotArmPickPlaceEnv

env = RobotArmPickPlaceEnv(render=True)

model = PPO.load("robot_arm_model.zip")


obs, _ = env.reset()

while True:
    action, _ = model.predict(obs)
    obs, reward, terminated, truncated, info = env.step(action)

    if terminated or truncated:
        obs, _ = env.reset()

    time.sleep(1. / 240.) 
