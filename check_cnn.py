from stable_baselines3 import PPO

model = PPO.load("robot_arm_model.zip")  # adjust if your zip name is different
print(model.policy.features_extractor)
