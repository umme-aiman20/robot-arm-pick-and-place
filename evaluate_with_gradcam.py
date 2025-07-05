import torch
import numpy as np
import cv2
import matplotlib.pyplot as plt
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecTransposeImage
from env.robot_env import RobotArmPickPlaceEnv
import pybullet as p
import time

# ---- Grad-CAM logic ----
def generate_gradcam(model, observation, target_layer):
    observation = torch.tensor(observation).float().unsqueeze(0)  # [1, 3, 84, 84]

    activations = []
    gradients = []

    def forward_hook(module, input, output):
        activations.append(output)

    def backward_hook(module, grad_input, grad_output):
        gradients.append(grad_output[0])

    handle_forward = target_layer.register_forward_hook(forward_hook)
    handle_backward = target_layer.register_backward_hook(backward_hook)

    features = model.policy.features_extractor(observation)
    logits = model.policy.action_net(features)
    predicted_action = torch.argmax(logits)
    logits[0, predicted_action].backward()

    grad = gradients[0][0]
    act = activations[0][0]
    weights = torch.mean(grad, dim=(1, 2))
    cam = torch.sum(weights[:, None, None] * act, dim=0)
    cam = torch.relu(cam)
    cam -= cam.min()
    cam /= cam.max() + 1e-8
    cam = cam.detach().cpu().numpy()
    cam = cv2.resize(cam, (84, 84))

    handle_forward.remove()
    handle_backward.remove()

    return cam

# ---- Main Loop ----
env = DummyVecEnv([lambda: RobotArmPickPlaceEnv(render=True)])
env = VecTransposeImage(env)

model = PPO.load("robot_arm_model.zip")
target_cnn_layer = model.policy.features_extractor.cnn[4]

try:
    obs = env.reset()
    while True:
        if not p.isConnected():
            print("🔌 PyBullet window closed. Exiting loop.")
            break

        cam = generate_gradcam(model, obs[0], target_cnn_layer)
        action, _ = model.predict(obs, deterministic=True)

        try:
            obs, reward, done, _ = env.step(action)
        except Exception as e:
            print("⚠️ Step failed:", e)
            break

        if not p.isConnected():
            print("🔌 Disconnected after step. Exiting.")
            break

        frame = obs[0].transpose(1, 2, 0)
        frame = np.ascontiguousarray(frame)
        cam_colored = cv2.applyColorMap(np.uint8(255 * cam), cv2.COLORMAP_JET)
        overlayed = cv2.addWeighted(frame, 0.6, cam_colored, 0.4, 0)

        cv2.imshow("Grad-CAM Heatmap", overlayed)
        key = cv2.waitKey(10)
        if key & 0xFF == ord('q'):
            print("👋 Quit key pressed.")
            break

        if done:
            if p.isConnected():
                obs = env.reset()
            else:
                print("❌ Skip reset: PyBullet not connected.")
                break

except KeyboardInterrupt:
    print("\n⛔ KeyboardInterrupt received. Exiting gracefully.")

finally:
    print("🔻 Cleaning up...")
    env.close()
    if p.isConnected():
        p.disconnect()
    cv2.destroyAllWindows()
