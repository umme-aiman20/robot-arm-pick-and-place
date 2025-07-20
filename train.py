from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from env.robot_env import RobotArmPickPlaceEnv
from stable_baselines3.common.torch_layers import NatureCNN
from stable_baselines3.ppo import CnnPolicy

env = RobotArmPickPlaceEnv(render=False)
check_env(env, warn=True)  

model = PPO("CnnPolicy", env, verbose=1)
model.learn(total_timesteps=10000)
model.save("robot_arm_model")
env.close()
