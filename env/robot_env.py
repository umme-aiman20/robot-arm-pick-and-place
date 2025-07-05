import pybullet as p
import pybullet_data
import gymnasium as gym
from gymnasium import spaces
import numpy as np
import time
import os

class RobotArmPickPlaceEnv(gym.Env):
    def __init__(self, render=False):
        super(RobotArmPickPlaceEnv, self).__init__()
        self.render = render
        self.time_step = 1. / 240.

        # Image observation shape
        self.img_width = 84
        self.img_height = 84

        if self.render:
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        self.robot = None
        self.block = None
        self.end_effector_index = None

        # Action space: 7 joint positions (normalized)
        self.action_space = spaces.Box(low=-1, high=1, shape=(7,), dtype=np.float32)

        # Observation space: RGB image (84x84x3)
        self.observation_space = spaces.Box(
            low=0,
            high=255,
            shape=(self.img_height, self.img_width, 3),
            dtype=np.uint8
        )

        self.reset()

    def reset(self, seed=None, options=None):
    # ✅ Ensure PyBullet is connected
        if not p.isConnected():
            if self.render:
                p.connect(p.GUI)
            else:
                p.connect(p.DIRECT)

        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(self.time_step)
        p.loadURDF("plane.urdf")
        p.loadURDF("table/table.urdf", [0.5, 0, 0])

        self.robot = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)
        self.num_joints = p.getNumJoints(self.robot)
        self.end_effector_index = self.num_joints - 1

        # Place cube randomly
        x = np.random.uniform(0.6, 0.8)
        y = np.random.uniform(-0.1, 0.1)
        self.block = p.loadURDF("cube_small.urdf", [x, y, 0.65])

        for _ in range(50):
            p.stepSimulation()

        obs = self._get_obs()
        return obs, {}


    def _get_obs(self):
        view_matrix = p.computeViewMatrix(
            cameraEyePosition=[1.3, 0, 1],
            cameraTargetPosition=[0.5, 0, 0],
            cameraUpVector=[0, 0, 1]
        )
        proj_matrix = p.computeProjectionMatrixFOV(
            fov=60,
            aspect=float(self.img_width) / self.img_height,
            nearVal=0.1,
            farVal=3.1
        )

        try:
            _, _, px, _, _ = p.getCameraImage(
                width=self.img_width,
                height=self.img_height,
                viewMatrix=view_matrix,
                projectionMatrix=proj_matrix,
                renderer=p.ER_BULLET_HARDWARE_OPENGL
            )
        except Exception as e:
            print("Camera capture failed:", e)
            px = None

        if px is None:
            px = np.zeros((self.img_height, self.img_width, 4), dtype=np.uint8)

        rgb_array = np.reshape(px, (self.img_height, self.img_width, 4))
        rgb_array = rgb_array[:, :, :3]  # remove alpha channel

        return rgb_array

    def step(self, action):
        if not p.isConnected():
            print("🚨 Warning: Disconnected from physics server. Returning dummy values.")
            return np.zeros(self.observation_space.shape), 0.0, True, False, {}

        # Clip action in [-1, 1] and scale to joint positions
        target_positions = np.clip(action, -1, 1) * np.pi

        for i in range(7):
            p.setJointMotorControl2(
                self.robot, i, p.POSITION_CONTROL,
                targetPosition=target_positions[i], force=200
            )

        for _ in range(10):
            p.stepSimulation()
            if self.render:
                time.sleep(self.time_step)

        obs = self._get_obs()
        reward, done = self._compute_reward()
        return obs, float(reward), bool(done), False, {}


    def _compute_reward(self):
        if not p.isConnected():
            print("❌ Skipped reward calculation (disconnected).")
            return 0.0, True  # Safely end episode

        ee_state = p.getLinkState(self.robot, self.end_effector_index)[0]
        block_pos = p.getBasePositionAndOrientation(self.block)[0]

        dist = np.linalg.norm(np.array(ee_state) - np.array(block_pos))
        reward = -dist
        done = dist < 0.05
        if done:
            reward += 10
        return reward, done


    def close(self):
        p.disconnect()
